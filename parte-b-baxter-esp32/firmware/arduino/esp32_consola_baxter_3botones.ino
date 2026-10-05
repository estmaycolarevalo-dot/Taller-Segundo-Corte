// Consola de mandos para Baxter con 3 pulsadores (sin joystick)
// Envia por serial: POS,<x>,<y>,<z>,<grip>   grip: 1 abierta, 0 cerrada

const int PIN_BTN_SELECCIONAR = 27;
const int PIN_BTN_MENOS = 26;
const int PIN_BTN_MAS = 25;

const float RANGO_XY = 0.5;
const float Z_MIN = -0.3;
const float Z_MAX = 0.3;

const float VELOCIDAD = 0.15;
const unsigned long PERIODO_CICLO_MS = 50;
const unsigned long TIEMPO_PULSACION_LARGA_MS = 800;

unsigned long ultimoCiclo = 0;

float pos[3] = {0.0, 0.0, 0.0};
int ejeActivo = 0;
const char nombresEje[3] = {'X', 'Y', 'Z'};

bool gripAbierto = true;

bool estadoAnteriorSel = HIGH;
unsigned long inicioPulsacionSel = 0;
bool pulsacionLargaProcesada = false;
const unsigned long ANTIRREBOTE_MS = 40;
unsigned long ultimoCambioSel = 0;

float limiteMin(int eje) { return (eje == 2) ? Z_MIN : -RANGO_XY; }
float limiteMax(int eje) { return (eje == 2) ? Z_MAX : RANGO_XY; }

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_SELECCIONAR, INPUT_PULLUP);
  pinMode(PIN_BTN_MENOS, INPUT_PULLUP);
  pinMode(PIN_BTN_MAS, INPUT_PULLUP);
  delay(2000);
  Serial.println("ESP32 listo (consola Baxter, 3 pulsadores)");
}

void loop() {
  unsigned long ahora = millis();

  bool sel = digitalRead(PIN_BTN_SELECCIONAR);
  if (sel != estadoAnteriorSel && (ahora - ultimoCambioSel) > ANTIRREBOTE_MS) {
    ultimoCambioSel = ahora;
    if (sel == LOW) {
      inicioPulsacionSel = ahora;
      pulsacionLargaProcesada = false;
    } else {
      unsigned long duracion = ahora - inicioPulsacionSel;
      if (duracion < TIEMPO_PULSACION_LARGA_MS) {
        ejeActivo = (ejeActivo + 1) % 3;
        Serial.print("Eje activo: ");
        Serial.println(nombresEje[ejeActivo]);
      }
    }
    estadoAnteriorSel = sel;
  }
  if (sel == LOW && !pulsacionLargaProcesada && (ahora - inicioPulsacionSel) >= TIEMPO_PULSACION_LARGA_MS) {
    gripAbierto = !gripAbierto;
    pulsacionLargaProcesada = true;
    Serial.println(gripAbierto ? "Pinza: ABIERTA" : "Pinza: CERRADA");
  }

  if (ahora - ultimoCiclo >= PERIODO_CICLO_MS) {
    float delta = VELOCIDAD * (PERIODO_CICLO_MS / 1000.0);
    bool menos = digitalRead(PIN_BTN_MENOS) == LOW;
    bool mas = digitalRead(PIN_BTN_MAS) == LOW;

    if (menos && !mas) {
      pos[ejeActivo] -= delta;
    } else if (mas && !menos) {
      pos[ejeActivo] += delta;
    }
    pos[ejeActivo] = constrain(pos[ejeActivo], limiteMin(ejeActivo), limiteMax(ejeActivo));

    ultimoCiclo = ahora;
    Serial.printf("POS,%.3f,%.3f,%.3f,%d\n", pos[0], pos[1], pos[2], gripAbierto ? 1 : 0);
  }
}
