#define TINY_GSM_MODEM_SIM800 //Tipo de modem que estamos usando
#include <TinyGsmClient.h> //https://github.com/vshymanskyy/TinyGSM
#include <PubSubClient.h>

//Pinos SIM800L
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

//Pino Relê
#define RELAY_PIN 33

//Serial para se comunicar com o SIM800L
HardwareSerial SerialGSM(1);

//Objeto que controla o SIM800L
TinyGsm modemGSM(SerialGSM);

//Socket utilizado no cliente http
TinyGsmClientSecure gsmClient(modemGSM);

//Objeto que fará a comunicação com o Firebase

//Variáveis para guardar qual o evento e os dados que chegarem do Firebase
PubSubClient client(gsmClient);

const char* mqttTopic = "oioioi";
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;

void setup()
{
    //Inicializa a serial que usaremos no monitor serial
    Serial.begin(115200);

    //Coloca o pino do relê como saída
    pinMode(RELAY_PIN, OUTPUT);

    //Desliga o relê. Pela configuração do nosso relê: HIGH desliga e LOW liga
    digitalWrite(RELAY_PIN, HIGH);

    //Configura o SIM800L
    setupSIM800L();

    //Espera 2 segundos
    delay(2000);

    client.setServer(mqttServer, mqttPort);

}

void setupSIM800L()
{
    Serial.println("Setup SIM800L...");

    //Reseta o SIM800L
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    //Inicializa a comunicação serial com o SIM800L
    SerialGSM.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);

    //Mostra informações sobre o SIM800L
    Serial.println(modemGSM.getModemInfo());

    //Conecta à rede telefônica
    Serial.print("Waiting for network...");
    if (!modemGSM.waitForNetwork())
    {
        Serial.println(" fail");
        delay(10000);

        return;
    }

    Serial.println(" success");

    //Conecta a rede de dados
    Serial.print("Connecting gprs ");
    if (!modemGSM.gprsConnect("", "", ""))
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }

    Serial.println(" success");
    Serial.println("Setup SIM800L Success");
}

void loop(){
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  float temperature = random(-100,100);
  float humidity = random(-100,100);

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    String payload = "Temperature: " + String(temperature) + " °C, Humidity: " + String(humidity) + " %";
    client.publish(mqttTopic, payload.c_str());

    delay(5000);  // Adjust delay as needed
  }

}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}