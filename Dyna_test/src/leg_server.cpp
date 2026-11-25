// leg_server.cpp - pilotage d'une patte (3 Dynamixel AX-12A / AX-18F, protocole 1.0)
//
// Pilote les servos par leurs registres et recoit des ordres texte sur TCP (port 5555),
// une commande par ligne. Utilise par l'interface graphique gui/leg_gui.py.
//
//   ./bin/leg_server                 # IDs 12 13 14, port 5555, /dev/u2d2
//   ./bin/leg_server 12 13 14 5555   # IDs et port explicites
//
// Commandes (reponse "OK ..." ou "ERR ...") :
//   PING                         -> OK pong
//   SCAN                         -> OK 12:1 13:1 14:0  (1 = le servo repond)
//   STATE                        -> OK {json}  etat + reglages des 3 servos
//   GOAL id pos                  position cible 0..1023            (reg. 30)
//   GOALS p1 p2 p3               positions cibles des 3 servos (sync write)
//   SPEED id v                   vitesse de deplacement 0..1023    (reg. 32, 0 = max)
//   TORQUE id 0|1                couple ON/OFF                     (reg. 24)
//   TLIMIT id v                  limite de couple 0..1023          (reg. 34, RAM)
//   MARGIN id cw ccw             marges de compliance 0..255       (reg. 26/27)
//   SLOPE id cw ccw              pentes de compliance 1..254       (reg. 28/29)
//   PUNCH id v                   punch 0..1023                     (reg. 48)
//   LIMITS id cw ccw             butees d'angle 0..1023            (reg. 6/8, EEPROM)
//   WRITE id addr len val        ecriture brute (len = 1 ou 2)
//   READ id addr len             lecture brute  (len = 1 ou 2)
//   ESTOP                        couple OFF sur les 3 servos
//
// Arret (Ctrl+C, SIGTERM) : le couple des 3 servos est coupe avant de quitter.
//
// Reconnexion automatique : si l'U2D2 decroche et revient (parasites quand les servos
// tirent du courant -> nouveau /dev/ttyUSBx), le port /dev/u2d2 est rouvert tout seul.

#include <climits>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

// ----------------------------------------------------------------- configuration
#define DEVICENAME        "/dev/u2d2"   // nom fixe de l'U2D2 (regle udev 99-u2d2.rules)
#define BAUDRATE          1000000
#define PROTOCOL_VERSION  1.0
#define DEFAULT_PORT      5555

// Table de controle AX-12A / AX-18F
#define ADDR_CW_ANGLE_LIMIT    6    // 2 octets, EEPROM
#define ADDR_CCW_ANGLE_LIMIT   8    // 2 octets, EEPROM
#define ADDR_MAX_TORQUE       14    // 2 octets, EEPROM
#define ADDR_TORQUE_ENABLE    24
#define ADDR_CW_MARGIN        26
#define ADDR_CCW_MARGIN       27
#define ADDR_CW_SLOPE         28
#define ADDR_CCW_SLOPE        29
#define ADDR_GOAL_POSITION    30    // 2 octets
#define ADDR_MOVING_SPEED     32    // 2 octets
#define ADDR_TORQUE_LIMIT     34    // 2 octets
#define ADDR_PRESENT_POSITION 36    // 2 octets
#define ADDR_PRESENT_SPEED    38    // 2 octets
#define ADDR_PRESENT_LOAD     40    // 2 octets
#define ADDR_PRESENT_VOLTAGE  42
#define ADDR_PRESENT_TEMP     43
#define ADDR_MOVING           46
#define ADDR_PUNCH            48    // 2 octets
#define TABLE_SIZE            50    // adresses 0..49 lues d'un coup

static volatile sig_atomic_t g_running = 1;
static void on_signal(int) { g_running = 0; }

static dynamixel::PortHandler   *port = nullptr;
static dynamixel::PacketHandler *pkt  = nullptr;
static std::vector<int> ids;

// ----------------------------------------------------------------- port / reconnexion
#define FAIL_STREAK_REOPEN 8          // STATE consecutifs sans aucune reponse -> on rouvre le port
static std::string g_opened_target;   // ttyUSBx reellement ouvert
static int g_reconnects = 0;
static int g_fail_streak = 0;

static std::string current_target() {
  char buf[PATH_MAX];
  return realpath(DEVICENAME, buf) ? std::string(buf) : std::string();
}

static bool open_port() {
  if (port) { port->closePort(); delete port; port = nullptr; }
  port = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  if (!port->openPort()) return false;
  if (!port->setBaudRate(BAUDRATE)) return false;
  g_opened_target = current_target();
  return true;
}

// Verifie que le port ouvert est toujours le bon ; le rouvre sinon. "" = OK.
static std::string ensure_port(bool force = false) {
  std::string t = current_target();
  if (t.empty()) return "U2D2 absent (/dev/u2d2 introuvable)";
  if (force || t != g_opened_target) {
    if (!open_port()) return "reouverture de /dev/u2d2 impossible";
    g_reconnects++;
    g_fail_streak = 0;
    printf("Port rouvert (%s), reconnexion n%d\n", g_opened_target.c_str(), g_reconnects);
    fflush(stdout);
  }
  return "";
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int word(const uint8_t *t, int a) { return t[a] | (t[a + 1] << 8); }
// vitesse / charge : bits 0-9 = valeur, bit 10 = sens -> valeur signee
static int signed10(int raw) { int v = raw & 0x3FF; return (raw & 0x400) ? -v : v; }

static bool is_known(int id) {
  for (int i : ids) if (i == id) return true;
  return false;
}

static std::string comm_err(int res, uint8_t err) {
  if (res != COMM_SUCCESS) return pkt->getTxRxResult(res);
  if (err) return pkt->getRxPacketError(err);
  return "";
}

static std::string w1(int id, int addr, int val) {
  uint8_t err = 0;
  int res = pkt->write1ByteTxRx(port, id, addr, (uint8_t)val, &err);
  return comm_err(res, err);
}

static std::string w2(int id, int addr, int val) {
  uint8_t err = 0;
  int res = pkt->write2ByteTxRx(port, id, addr, (uint16_t)val, &err);
  return comm_err(res, err);
}

static std::string ok_or(const std::string &e, const std::string &ok = "OK") {
  return e.empty() ? ok : "ERR " + e;
}

static void torque_all(int on) {
  for (int id : ids) w1(id, ADDR_TORQUE_ENABLE, on);
}

// Etat complet des servos en JSON (une ligne)
static std::string state_json() {
  std::ostringstream o;
  std::string perr = ensure_port();
  std::string tgt = g_opened_target.substr(g_opened_target.find_last_of('/') + 1);
  o << "{\"port\":\"" << tgt << "\",\"reconnects\":" << g_reconnects;
  if (!perr.empty()) {
    o << ",\"port_error\":\"" << perr << "\",\"servos\":[]}";
    return o.str();
  }
  o << ",\"servos\":[";
  int failed = 0;
  for (size_t k = 0; k < ids.size(); k++) {
    int id = ids[k];
    uint8_t t[TABLE_SIZE] = {0};
    uint8_t err = 0;
    int res = pkt->readTxRx(port, id, 0, TABLE_SIZE, t, &err);
    if (k) o << ",";
    if (res != COMM_SUCCESS) {
      failed++;
      o << "{\"id\":" << id << ",\"ok\":false,\"msg\":\"" << pkt->getTxRxResult(res) << "\"}";
      continue;
    }
    o << "{\"id\":" << id << ",\"ok\":true"
      << ",\"err\":" << (int)err
      << ",\"model\":" << word(t, 0)
      << ",\"pos\":" << word(t, ADDR_PRESENT_POSITION)
      << ",\"goal\":" << word(t, ADDR_GOAL_POSITION)
      << ",\"speed\":" << signed10(word(t, ADDR_PRESENT_SPEED))
      << ",\"load\":" << signed10(word(t, ADDR_PRESENT_LOAD))
      << ",\"volt\":" << (int)t[ADDR_PRESENT_VOLTAGE]
      << ",\"temp\":" << (int)t[ADDR_PRESENT_TEMP]
      << ",\"moving\":" << (int)t[ADDR_MOVING]
      << ",\"torque\":" << (int)t[ADDR_TORQUE_ENABLE]
      << ",\"mspeed\":" << word(t, ADDR_MOVING_SPEED)
      << ",\"tlimit\":" << word(t, ADDR_TORQUE_LIMIT)
      << ",\"maxtorque\":" << word(t, ADDR_MAX_TORQUE)
      << ",\"cw_limit\":" << word(t, ADDR_CW_ANGLE_LIMIT)
      << ",\"ccw_limit\":" << word(t, ADDR_CCW_ANGLE_LIMIT)
      << ",\"cw_margin\":" << (int)t[ADDR_CW_MARGIN]
      << ",\"ccw_margin\":" << (int)t[ADDR_CCW_MARGIN]
      << ",\"cw_slope\":" << (int)t[ADDR_CW_SLOPE]
      << ",\"ccw_slope\":" << (int)t[ADDR_CCW_SLOPE]
      << ",\"punch\":" << word(t, ADDR_PUNCH)
      << "}";
  }
  o << "]}";
  // Aucun servo ne repond plusieurs fois de suite : on rouvre le port par precaution
  // (cas d'un U2D2 reconnecte sous le meme nom ttyUSBx).
  if (failed == (int)ids.size()) {
    if (++g_fail_streak >= FAIL_STREAK_REOPEN) ensure_port(true);
  } else {
    g_fail_streak = 0;
  }
  return o.str();
}

static std::string goals(const std::vector<int> &p) {
  if (p.size() != ids.size()) return "ERR il faut " + std::to_string(ids.size()) + " positions";
  dynamixel::GroupSyncWrite sw(port, pkt, ADDR_GOAL_POSITION, 2);
  for (size_t k = 0; k < ids.size(); k++) {
    int v = clampi(p[k], 0, 1023);
    uint8_t b[2] = {DXL_LOBYTE(v), DXL_HIBYTE(v)};
    sw.addParam(ids[k], b);
  }
  int res = sw.txPacket();
  return res == COMM_SUCCESS ? "OK" : "ERR " + std::string(pkt->getTxRxResult(res));
}

// Traite une ligne de commande, renvoie la reponse (sans \n)
static std::string handle(const std::string &line) {
  std::istringstream in(line);
  std::string cmd;
  in >> cmd;
  for (auto &c : cmd) c = toupper(c);
  if (cmd.empty()) return "ERR commande vide";

  if (cmd == "PING") return "OK pong";
  if (cmd == "STATE") return "OK " + state_json();
  { std::string e = ensure_port(); if (!e.empty()) return "ERR " + e; }
  if (cmd == "ESTOP") { torque_all(0); return "OK couple coupe"; }

  if (cmd == "SCAN") {
    std::string r = "OK";
    for (int id : ids) {
      uint16_t model = 0; uint8_t err = 0;
      int res = pkt->ping(port, id, &model, &err);
      r += " " + std::to_string(id) + ":" + (res == COMM_SUCCESS ? "1" : "0");
    }
    return r;
  }

  if (cmd == "GOALS") {
    std::vector<int> p; int v;
    while (in >> v) p.push_back(v);
    return goals(p);
  }

  int id;
  if (!(in >> id)) return "ERR id manquant";
  if (!is_known(id)) return "ERR id " + std::to_string(id) + " non gere par ce serveur";
  int a = 0, b = 0, c = 0;

  if (cmd == "GOAL")   { if (!(in >> a)) return "ERR position manquante"; return ok_or(w2(id, ADDR_GOAL_POSITION, clampi(a, 0, 1023))); }
  if (cmd == "SPEED")  { if (!(in >> a)) return "ERR vitesse manquante";  return ok_or(w2(id, ADDR_MOVING_SPEED, clampi(a, 0, 1023))); }
  if (cmd == "TORQUE") { if (!(in >> a)) return "ERR 0/1 manquant";       return ok_or(w1(id, ADDR_TORQUE_ENABLE, a ? 1 : 0)); }
  if (cmd == "TLIMIT") { if (!(in >> a)) return "ERR valeur manquante";   return ok_or(w2(id, ADDR_TORQUE_LIMIT, clampi(a, 0, 1023))); }
  if (cmd == "PUNCH")  { if (!(in >> a)) return "ERR valeur manquante";   return ok_or(w2(id, ADDR_PUNCH, clampi(a, 0, 1023))); }
  if (cmd == "MARGIN") {
    if (!(in >> a >> b)) return "ERR cw ccw manquants";
    std::string e = w1(id, ADDR_CW_MARGIN, clampi(a, 0, 255));
    if (e.empty()) e = w1(id, ADDR_CCW_MARGIN, clampi(b, 0, 255));
    return ok_or(e);
  }
  if (cmd == "SLOPE") {
    if (!(in >> a >> b)) return "ERR cw ccw manquants";
    std::string e = w1(id, ADDR_CW_SLOPE, clampi(a, 1, 254));
    if (e.empty()) e = w1(id, ADDR_CCW_SLOPE, clampi(b, 1, 254));
    return ok_or(e);
  }
  if (cmd == "LIMITS") {
    if (!(in >> a >> b)) return "ERR cw ccw manquants";
    a = clampi(a, 0, 1023); b = clampi(b, 0, 1023);
    if (a >= b) return "ERR il faut cw < ccw (0/0 = mode roue, refuse ici)";
    std::string e = w2(id, ADDR_CW_ANGLE_LIMIT, a);
    if (e.empty()) e = w2(id, ADDR_CCW_ANGLE_LIMIT, b);
    return ok_or(e);
  }
  if (cmd == "WRITE") {
    if (!(in >> a >> b >> c)) return "ERR addr len val manquants";
    if (a < 3 || a > 49) return "ERR adresse hors 3..49 (ID/modele proteges)";
    if (a == 3 || a == 4) return "ERR ID et baudrate non modifiables ici";
    return ok_or(b == 2 ? w2(id, a, c) : w1(id, a, c));
  }
  if (cmd == "READ") {
    if (!(in >> a >> b)) return "ERR addr len manquants";
    uint8_t err = 0; int res;
    if (b == 2) { uint16_t v = 0; res = pkt->read2ByteTxRx(port, id, a, &v, &err); c = v; }
    else        { uint8_t  v = 0; res = pkt->read1ByteTxRx(port, id, a, &v, &err); c = v; }
    std::string e = comm_err(res, err);
    return e.empty() ? "OK " + std::to_string(c) : "ERR " + e;
  }
  return "ERR commande inconnue : " + cmd;
}

int main(int argc, char **argv) {
  int tcp_port = DEFAULT_PORT;
  if (argc >= 4) {
    for (int i = 1; i <= 3; i++) ids.push_back(atoi(argv[i]));
    if (argc >= 5) tcp_port = atoi(argv[4]);
  } else {
    ids = {12, 13, 14};
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);

  pkt = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  if (!open_port()) {
    fprintf(stderr, "Impossible d'ouvrir %s a %d bauds (U2D2 branche ?)\n", DEVICENAME, BAUDRATE);
    return 1;
  }
  printf("Port %s (%s) ouvert a %d bauds. Servos :", DEVICENAME, g_opened_target.c_str(), BAUDRATE);
  for (int id : ids) printf(" %d", id);
  printf("\n%s\n", handle("SCAN").c_str());

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(tcp_port);
  if (bind(srv, (sockaddr *)&addr, sizeof(addr)) < 0 || listen(srv, 1) < 0) {
    perror("bind/listen"); return 1;
  }
  printf("En attente de l'interface sur le port TCP %d (Ctrl+C pour quitter)...\n", tcp_port);
  fflush(stdout);

  int cli = -1;
  std::string buf;
  while (g_running) {
    pollfd pf = { cli >= 0 ? cli : srv, POLLIN, 0 };
    int n = poll(&pf, 1, 200);
    if (n <= 0) continue;

    if (cli < 0) {
      sockaddr_in ca{}; socklen_t cl = sizeof(ca);
      cli = accept(srv, (sockaddr *)&ca, &cl);
      if (cli >= 0) {
        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        printf("Interface connectee depuis %s\n", inet_ntoa(ca.sin_addr));
        fflush(stdout);
        buf.clear();
      }
      continue;
    }

    char tmp[1024];
    ssize_t r = recv(cli, tmp, sizeof(tmp), 0);
    if (r <= 0) {
      printf("Interface deconnectee\n"); fflush(stdout);
      close(cli); cli = -1; continue;
    }
    buf.append(tmp, r);
    size_t nl;
    while ((nl = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, nl);
      buf.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::string rep = handle(line) + "\n";
      send(cli, rep.data(), rep.size(), 0);
    }
  }

  printf("\nArret : couple coupe sur les servos.\n");
  torque_all(0);
  if (cli >= 0) close(cli);
  close(srv);
  port->closePort();
  return 0;
}
