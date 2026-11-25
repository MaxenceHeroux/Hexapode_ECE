// leg_server.cpp - pilotage des Dynamixel de l'hexapode (AX-12A / AX-18F, protocole 1.0)
//
// Pilote les servos par leurs registres et recoit des ordres texte sur TCP (port 5555),
// une commande par ligne. Utilise par gui/leg_gui.py et gui/test_torque.py.
//
//   ./bin/leg_server                     # IDs 0..17 (les 6 pattes), port 5555, /dev/u2d2
//   ./bin/leg_server 12 13 14            # seulement ces IDs
//   ./bin/leg_server --port 5556 0 1 2   # autre port
//
// Les servos qui ne repondent pas sont marques absents : ils ne sont plus lus a chaque
// STATE (sinon chaque absent coute ~35 ms), un seul absent est re-teste par STATE.
//
// Commandes (reponse "OK ..." ou "ERR ...") :
//   PING                         -> OK pong
//   IDS id id ...                remplace la liste des servos geres (puis SCAN)
//   SCAN                         -> OK 12:1 13:1 14:0  (1 = le servo repond)
//   STATE                        -> OK {json}  etat + reglages de tous les servos
//   GOAL id pos                  position cible 0..1023            (reg. 30)
//   GOALS p1 p2 ...              positions cibles de TOUS les servos, dans l'ordre (sync write)
//   GOALSID id pos id pos ...    positions cibles des servos cites (sync write)
//   GOALSPD id pos spd ...       position cible + vitesse (reg. 30-33) des servos cites, en un
//                                seul sync write : mouvement fluide d'une consigne a la suivante
//   SYNCW addr len id val ...    sync write brut d'un registre RAM (addr 24..49, len 1 ou 2),
//                                ex. SYNCW 28 2 13 2056 = pentes CW/CCW 8/8 du servo 13
//   SPEED id v                   vitesse de deplacement 0..1023    (reg. 32, 0 = max)
//   TORQUE id 0|1                couple ON/OFF                     (reg. 24)
//   TLIMIT id v                  limite de couple 0..1023          (reg. 34, RAM)
//   MARGIN id cw ccw             marges de compliance 0..255       (reg. 26/27)
//   SLOPE id cw ccw              pentes de compliance 1..254       (reg. 28/29)
//   PUNCH id v                   punch 0..1023                     (reg. 48)
//   LIMITS id cw ccw             butees d'angle 0..1023            (reg. 6/8, EEPROM)
//   WRITE id addr len val        ecriture brute (len = 1 ou 2)
//   READ id addr len             lecture brute  (len = 1 ou 2)
//   ESTOP                        couple OFF sur tous les servos
//   RESET [id|ALL]               sortie du mode securite (surcharge, surchauffe...) :
//                                consigne = position actuelle, limite de couple (34)
//                                = couple max (14), couple ON. Sans id : tous les servos.
//
// Arret (Ctrl+C, SIGTERM) : le couple de tous les servos est coupe avant de quitter.
//
// Reglages au demarrage : les commandes du fichier Dyna_test/leg_settings.txt (a cote du
// dossier bin/) sont executees au lancement (registres RAM des AX, perdus hors tension).
//
// Reconnexion automatique : si l'U2D2 decroche et revient (parasites quand les servos
// tirent du courant -> nouveau /dev/ttyUSBx), le port /dev/u2d2 est rouvert tout seul.

#include <algorithm>
#include <array>
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
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

// ----------------------------------------------------------------- configuration
#define DEVICENAME        "/dev/u2d2"   // nom fixe de l'U2D2 (regle udev 99-u2d2.rules)
#define BAUDRATE          1000000
#define PROTOCOL_VERSION  1.0
#define DEFAULT_PORT      5555
#define MAX_CLIENTS       4             // interface + test_torque + nc... en meme temps

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
static std::map<int, bool> present;     // le servo a-t-il repondu au dernier ping / lecture ?
static size_t g_probe = 0;              // tourniquet de re-test des absents

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

// Sortie du mode securite d'un AX. Apres une alarme (surcharge, surchauffe...), le servo
// coupe son couple et met sa limite de couple (reg. 34) a 0 : il reste mou meme si on
// reactive le couple. Les AX n'ont pas d'instruction "reboot" : on remet les registres.
// Les ecritures ignorent l'octet d'erreur du servo (il reste leve pendant l'alarme),
// seules les erreurs de communication comptent.
static std::string reset_servo(int id) {
  uint8_t err = 0;
  uint16_t pos = 0, maxt = 0;
  int res = pkt->read2ByteTxRx(port, id, ADDR_PRESENT_POSITION, &pos, &err);
  if (res != COMM_SUCCESS) return "ID " + std::to_string(id) + " : " + pkt->getTxRxResult(res);
  res = pkt->read2ByteTxRx(port, id, ADDR_MAX_TORQUE, &maxt, &err);
  if (res != COMM_SUCCESS) return "ID " + std::to_string(id) + " : " + pkt->getTxRxResult(res);
  if (maxt == 0) maxt = 1023;
  // 1) consigne = position actuelle (pas de saut au rearmement)
  res = pkt->write2ByteTxRx(port, id, ADDR_GOAL_POSITION, pos, &err);
  // 2) limite de couple remise au couple max
  if (res == COMM_SUCCESS) res = pkt->write2ByteTxRx(port, id, ADDR_TORQUE_LIMIT, maxt, &err);
  // 3) couple ON
  if (res == COMM_SUCCESS) res = pkt->write1ByteTxRx(port, id, ADDR_TORQUE_ENABLE, 1, &err);
  if (res != COMM_SUCCESS) return "ID " + std::to_string(id) + " : " + pkt->getTxRxResult(res);
  printf("RESET ID %d : consigne %d, limite de couple %d, couple ON\n", id, pos, maxt);
  fflush(stdout);
  return "";
}

static bool ping_id(int id) {
  uint16_t model = 0; uint8_t err = 0;
  bool ok = pkt->ping(port, id, &model, &err) == COMM_SUCCESS;
  present[id] = ok;
  return ok;
}

static std::vector<int> present_ids() {
  std::vector<int> v;
  for (int id : ids) if (present[id]) v.push_back(id);
  return v;
}

static void torque_all(int on) {
  for (int id : present_ids()) w1(id, ADDR_TORQUE_ENABLE, on);
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
  // re-teste un seul servo absent par STATE (tourniquet)
  std::vector<int> absent;
  for (int id : ids) if (!present[id]) absent.push_back(id);
  if (!absent.empty()) ping_id(absent[g_probe++ % absent.size()]);

  o << ",\"servos\":[";
  int failed = 0, polled = 0;
  for (size_t k = 0; k < ids.size(); k++) {
    int id = ids[k];
    if (k) o << ",";
    if (!present[id]) {
      o << "{\"id\":" << id << ",\"ok\":false,\"msg\":\"absent\"}";
      continue;
    }
    polled++;
    uint8_t t[TABLE_SIZE] = {0};
    uint8_t err = 0;
    int res = pkt->readTxRx(port, id, 0, TABLE_SIZE, t, &err);
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
  if (polled > 0 && failed == polled) {
    if (++g_fail_streak >= FAIL_STREAK_REOPEN) {
      ensure_port(true);
      for (int id : ids) ping_id(id);          // re-scan complet apres reouverture
    }
  } else {
    g_fail_streak = 0;
  }
  return o.str();
}

// Sync write des consignes : paires (id, position)
static std::string sync_goals(const std::vector<std::pair<int, int>> &g) {
  if (g.empty()) return "ERR aucune consigne";
  dynamixel::GroupSyncWrite sw(port, pkt, ADDR_GOAL_POSITION, 2);
  for (auto &kv : g) {
    if (!is_known(kv.first)) return "ERR id " + std::to_string(kv.first) + " non gere par ce serveur";
    int v = clampi(kv.second, 0, 1023);
    uint8_t b[2] = {DXL_LOBYTE(v), DXL_HIBYTE(v)};
    sw.addParam(kv.first, b);
  }
  int res = sw.txPacket();
  return res == COMM_SUCCESS ? "OK" : "ERR " + std::string(pkt->getTxRxResult(res));
}

// Sync write : position cible + vitesse de deplacement (registres 30..33)
static std::string sync_goal_speed(const std::vector<std::array<int, 3>> &g) {
  if (g.empty()) return "ERR aucune consigne";
  dynamixel::GroupSyncWrite sw(port, pkt, ADDR_GOAL_POSITION, 4);
  for (auto &v : g) {
    if (!is_known(v[0])) return "ERR id " + std::to_string(v[0]) + " non gere par ce serveur";
    int pos = clampi(v[1], 0, 1023), spd = clampi(v[2], 0, 1023);
    uint8_t b[4] = {DXL_LOBYTE(pos), DXL_HIBYTE(pos), DXL_LOBYTE(spd), DXL_HIBYTE(spd)};
    sw.addParam(v[0], b);
  }
  int res = sw.txPacket();
  return res == COMM_SUCCESS ? "OK" : "ERR " + std::string(pkt->getTxRxResult(res));
}

// Sync write brut d'un registre RAM (1 ou 2 octets) sur plusieurs servos
static std::string sync_raw(int addr, int len, const std::vector<std::pair<int, int>> &g) {
  if (addr < 24 || addr + len - 1 > 49) return "ERR adresse hors 24..49 (RAM seulement)";
  if (len != 1 && len != 2) return "ERR len = 1 ou 2";
  if (g.empty()) return "ERR aucune valeur";
  dynamixel::GroupSyncWrite sw(port, pkt, addr, len);
  for (auto &kv : g) {
    if (!is_known(kv.first)) return "ERR id " + std::to_string(kv.first) + " non gere par ce serveur";
    int v = clampi(kv.second, 0, len == 1 ? 255 : 65535);
    uint8_t b[2] = {DXL_LOBYTE(v), DXL_HIBYTE(v)};
    sw.addParam(kv.first, b);
  }
  int res = sw.txPacket();
  return res == COMM_SUCCESS ? "OK" : "ERR " + std::string(pkt->getTxRxResult(res));
}

static std::string scan() {
  std::string r = "OK";
  for (int id : ids) r += " " + std::to_string(id) + ":" + (ping_id(id) ? "1" : "0");
  return r;
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

  if (cmd == "SCAN") return scan();

  if (cmd == "IDS") {
    std::vector<int> nv; std::set<int> seen; int v;
    while (in >> v) {
      if (v < 0 || v > 252) return "ERR id " + std::to_string(v) + " hors 0..252";
      if (seen.insert(v).second) nv.push_back(v);
    }
    if (nv.empty() || nv.size() > 40) return "ERR il faut 1 a 40 IDs";
    ids = nv;
    present.clear();
    return scan();
  }

  if (cmd == "RESET") {
    std::string who;
    in >> who;
    std::vector<int> targets;
    if (who.empty() || who == "ALL" || who == "all") targets = present_ids();
    else {
      int id = atoi(who.c_str());
      if (!is_known(id)) return "ERR id " + who + " non gere par ce serveur";
      targets.push_back(id);
    }
    std::string errs;
    for (int id : targets) {
      std::string e = reset_servo(id);
      if (!e.empty()) errs += (errs.empty() ? "" : " ; ") + e;
    }
    return errs.empty() ? "OK securite reinitialisee" : "ERR " + errs;
  }

  if (cmd == "GOALS") {                 // toutes les consignes, dans l'ordre de la liste
    std::vector<int> p; int v;
    while (in >> v) p.push_back(v);
    if (p.size() != ids.size()) return "ERR il faut " + std::to_string(ids.size()) + " positions";
    std::vector<std::pair<int, int>> g;
    for (size_t k = 0; k < ids.size(); k++) g.push_back({ids[k], p[k]});
    return sync_goals(g);
  }

  if (cmd == "GOALSID") {               // paires id position
    std::vector<std::pair<int, int>> g; int a, b;
    while (in >> a >> b) g.push_back({a, b});
    return sync_goals(g);
  }

  if (cmd == "GOALSPD") {               // triplets id position vitesse
    std::vector<std::array<int, 3>> g; int a, b, c;
    while (in >> a >> b >> c) g.push_back({a, b, c});
    return sync_goal_speed(g);
  }

  if (cmd == "SYNCW") {                 // addr len puis paires id valeur
    int addr, len;
    if (!(in >> addr >> len)) return "ERR addr len manquants";
    std::vector<std::pair<int, int>> g; int a, b;
    while (in >> a >> b) g.push_back({a, b});
    return sync_raw(addr, len, g);
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

// Execute les commandes de leg_settings.txt (dossier parent de bin/), lignes # ignorees.
static void apply_settings_file() {
  char exe[PATH_MAX];
  ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n <= 0) return;
  exe[n] = 0;
  std::string dir(exe);
  dir = dir.substr(0, dir.find_last_of('/'));            // .../Dyna_test/bin
  dir = dir.substr(0, dir.find_last_of('/'));            // .../Dyna_test
  std::string path = dir + "/leg_settings.txt";
  FILE *fp = fopen(path.c_str(), "r");
  if (!fp) { printf("Pas de %s (reglages par defaut des servos)\n", path.c_str()); return; }
  printf("Reglages de %s :\n", path.c_str());
  char line[256];
  while (fgets(line, sizeof(line), fp)) {
    std::string l(line);
    while (!l.empty() && (l.back() == '\n' || l.back() == '\r' || l.back() == ' ')) l.pop_back();
    size_t p = l.find_first_not_of(" \t");
    if (p == std::string::npos || l[p] == '#') continue;
    printf("  %-24s -> %s\n", l.c_str() + p, handle(l.substr(p)).c_str());
  }
  fclose(fp);
  fflush(stdout);
}

int main(int argc, char **argv) {
  int tcp_port = DEFAULT_PORT;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) tcp_port = atoi(argv[++i]);
    else ids.push_back(atoi(argv[i]));
  }
  if (ids.empty()) for (int i = 0; i < 18; i++) ids.push_back(i);   // les 6 pattes

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
  apply_settings_file();

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(tcp_port);
  if (bind(srv, (sockaddr *)&addr, sizeof(addr)) < 0 || listen(srv, 4) < 0) {
    perror("bind/listen"); return 1;
  }
  printf("En attente des clients sur le port TCP %d (jusqu'a %d en meme temps, Ctrl+C pour quitter)...\n",
         tcp_port, MAX_CLIENTS);
  fflush(stdout);

  // Plusieurs clients en meme temps (ex. interface + test_torque) : les commandes sont
  // traitees une par une dans ce seul thread, donc jamais deux acces simultanes au bus.
  struct Client { int fd; std::string buf; std::string from; };
  std::vector<Client> clients;
  while (g_running) {
    std::vector<pollfd> pf;
    pf.push_back({srv, POLLIN, 0});
    for (auto &c : clients) pf.push_back({c.fd, POLLIN, 0});
    int n = poll(pf.data(), pf.size(), 200);
    if (n <= 0) continue;

    if (pf[0].revents & POLLIN) {
      sockaddr_in ca{}; socklen_t cl = sizeof(ca);
      int fd = accept(srv, (sockaddr *)&ca, &cl);
      if (fd >= 0) {
        if ((int)clients.size() >= MAX_CLIENTS) {
          const char *m = "ERR trop de clients connectes\n";
          send(fd, m, strlen(m), 0);
          close(fd);
        } else {
          setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
          clients.push_back({fd, "", inet_ntoa(ca.sin_addr)});
          printf("Client connecte depuis %s (%zu connecte(s))\n", clients.back().from.c_str(), clients.size());
          fflush(stdout);
        }
      }
    }

    for (size_t i = 1; i < pf.size(); i++) {
      if (!(pf[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      Client &c = clients[i - 1];
      char tmp[1024];
      ssize_t r = recv(c.fd, tmp, sizeof(tmp), 0);
      if (r <= 0) {
        close(c.fd);
        c.fd = -1;
        continue;
      }
      c.buf.append(tmp, r);
      size_t nl;
      while ((nl = c.buf.find('\n')) != std::string::npos) {
        std::string line = c.buf.substr(0, nl);
        c.buf.erase(0, nl + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::string rep = handle(line) + "\n";
        send(c.fd, rep.data(), rep.size(), 0);
      }
    }
    size_t before = clients.size();
    clients.erase(std::remove_if(clients.begin(), clients.end(), [](const Client &c) { return c.fd < 0; }),
                  clients.end());
    if (clients.size() != before) {
      printf("Client deconnecte (%zu connecte(s))\n", clients.size());
      fflush(stdout);
    }
  }

  printf("\nArret : couple coupe sur les servos.\n");
  torque_all(0);
  for (auto &c : clients) close(c.fd);
  close(srv);
  port->closePort();
  return 0;
}
