// Consola de 3 pulsadores para Atlas: mano derecha, mano izquierda y cuerpo
// SELECCIONAR corto: cambia el eje. SELECCIONAR largo: cambia el grupo
// Envia por serial: POS3,rx,ry,rz,lx,ly,lz,bx,by,giro

const int PIN_BTN_SELECCIONAR = 27;
const int PIN_BTN_MENOS = 26;
const int PIN_BTN_MAS = 25;

const char* nombresGrupo[3] = {"MANO DERECHA", "MANO IZQUIERDA", "CUERPO"};
const char* nombresEje[3][3] = {{"X", "Y", "Z"}, {"X", "Y", "Z"}, {"X", "Y", "GIRO"}};

float limMin[3][3] = {{-0.2, -0.2, -0.2}, {-0.2, -0.2, -0.2}, {-1.5, -1.5, -3.14}};
float limMax[3][3] = {{0.2, 0.2, 0.2}, {0.2, 0.2, 0.2}, {1.5, 1.5, 3.14}};
float velocidad[3][3] = {{0.1, 0.1, 0.1}, {0.1, 0.1, 0.1}, {0.3, 0.3, 0.8}};

const unsigned long PERIODO_CICLO_MS = 50;
const unsigned long TIEMPO_PULSACION_LARGA_MS = 800;
const unsigned long ANTIRREBOTE_MS = 40;

float pos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
int grupo = 0;
int eje = 0;

unsigned long ultimoCiclo = 0;
bool estadoAnteriorSel = HIGH;
unsigned long inicioPulsacionSel = 0;
bool pulsacionLargaProcesada = false;
unsigned long ultimoCambioSel = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_SELECCIONAR, INPUT_PULLUP);
  pinMode(PIN_BTN_MENOS, INPUT_PULLUP);
  pinMode(PIN_BTN_MAS, INPUT_PULLUP);
  delay(2000);
  Serial.println("ESP32 listo (consola Atlas, 3 pulsadores)");
}

void loop() {
  unsigned long ahora = millis();

  bool sel = digitalRead(PIN_BTN_SELECCIONAR);
  if (sel != estadoAnteriorSel && (ahora - ultimoCambioSel) > ANTIRREBOTE_MS) {
    ultimoCambioSel = ahora;
    if (sel == LOW) {
      inicioPulsacionSel = ahora;
      pulsacionLargaProcesada = false;
    } else if (ahora - inicioPulsacionSel < TIEMPO_PULSACION_LARGA_MS) {
      eje = (eje + 1) % 3;
      Serial.printf("Grupo: %s, eje: %s\n", nombresGrupo[grupo], nombresEje[grupo][eje]);
    }
    estadoAnteriorSel = sel;
  }
  if (sel == LOW && !pulsacionLargaProcesada && (ahora - inicioPulsacionSel) >= TIEMPO_PULSACION_LARGA_MS) {
    grupo = (grupo + 1) % 3;
    eje = 0;
    pulsacionLargaProcesada = true;
    Serial.printf("Grupo: %s, eje: %s\n", nombresGrupo[grupo], nombresEje[grupo][eje]);
  }

  if (ahora - ultimoCiclo >= PERIODO_CICLO_MS) {
    float delta = velocidad[grupo][eje] * (PERIODO_CICLO_MS / 1000.0);
    bool menos = digitalRead(PIN_BTN_MENOS) == LOW;
    bool mas = digitalRead(PIN_BTN_MAS) == LOW;

    if (menos && !mas) {
      pos[grupo][eje] -= delta;
    } else if (mas && !menos) {
      pos[grupo][eje] += delta;
    }
    pos[grupo][eje] = constrain(pos[grupo][eje], limMin[grupo][eje], limMax[grupo][eje]);

    ultimoCiclo = ahora;
    Serial.printf("POS3,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                  pos[0][0], pos[0][1], pos[0][2],
                  pos[1][0], pos[1][1], pos[1][2],
                  pos[2][0], pos[2][1], pos[2][2]);
  }
}
