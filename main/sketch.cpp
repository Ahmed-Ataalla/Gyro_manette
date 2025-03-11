#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include "sdkconfig.h"

// déclaration des pins
#define PIN_PWM_A_D 33
#define PIN_PWM_B_D 32
#define PIN_PWM_A_G 26
#define PIN_PWM_B_G 25
#define PIN_VBATT 34
#define PIN_CODEUR_A_D 19
#define PIN_CODEUR_B_D 18
#define PIN_CODEUR_A_G 17
#define PIN_CODEUR_B_G 16
#define PIN_LED 4

#define GAIN_PONT_DIV 0.2186  // gain pont diviseur de tension

// paramètres PWM
#define FREQUENCE 20000  // 20kHz
#define RESOLUTION 10    // 10 bits
// Canaux PWM
#define CANAL_PWM_A_D 0
#define CANAL_PWM_B_D 1
#define CANAL_PWM_A_G 2
#define CANAL_PWM_B_G 3

#define NB_TICK_TOURCOMPLET 680
#define RAYON_ROUE 32.5  // en m

// bits des boutons
#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_X 2
#define BUTTON_Y 3
#define BUTTON_LB 4
#define BUTTON_RB 5
#define BUTTON_STICK_L 8
#define BUTTON_STICK_R 9

char FlagCalcul = 0;

float Te = 5;     // en ms
float Tau = 325;  // en ms

// position angulaire calculé avec l'acceleration non filtré et filtré le filtré le filtre complémentaire
float theta_acc = 0, theta_acc_F, theta_acc_F_p = 0;
// position angulaire calculé avec la vitesse de rotation non filtré et filtré par le filtre complémentaire
float theta_w = 0, theta_w_F, theta_w_F_p = 0;
float theta_somme;
// coefficient du filtre complémentaire
float A_teta, B_teta;

float vit = 0, vitf, vit_F_P = 0;
float A_vit, B_vit;

float ec_mot_D, ec_mot_G;
float val_vbatt;  // mesure tension batterie

int compteur_codeur_D, compteur_codeur_G, compteur_codeur_D_p, compteur_codeur_G_p;
float vitesse_D, vitesse_G;  // vitesse mesuré avec le codeur

// coef asservisement pos
float theta_cons, theta0, erreur_theta = 0, erreur_theta_p = 0;  // theta cons 0.02...0.03 pour stabiliser
float ec_theta;
float kp_theta = 0, kd_theta = 0, ki_theta = 0;  // kp 1.5 kd 0.046
float Integrale_erreur_theta = 0;
float P_theta, D_theta, I_theta;

float kp_vit = 0, kd_vit = 0, ki_vit = 0;  // coeff
float P_vit_D, D_vit_D, I_vit_D;
float P_vit_G, D_vit_G, I_vit_G;
float vit_cons, erreur_vit_G, erreur_vit_G_p = 0;
float erreur_vit_D, erreur_vit_D_p = 0;
float ec_vitt_G, ec_vitt_D, ec_vitt_cons;
float D_vit_D_F, D_vit_D_F_p = 0, D_vit_G_F, D_vit_G_F_p = 0;
float Tau_D = 20;

float vit_D_cons, vit_G_cons;
float dir;

short val_button;
bool val_button_A;
bool val_button_B;
bool val_button_X;
bool val_button_Y;
bool val_button_LB;
bool val_button_RB;
bool val_button_STICK_L;
bool val_button_STICK_R;

int etat_ctl = 0, etat_led = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// kp_theta=1.84, kd_theta=0.062, kd_vit= 0.005, kd_vit= 0.15, theta0=0.02, Tau_D=400

ESP32Encoder encoder_D;
ESP32Encoder encoder_G;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;  // Pour la lecture des variables

float saturation_ec_mot(float ec_mot);
float saturation_ec_vit(float ec_vit);

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(),
                          properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }

    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    // lis la valeur du stick gauche et boutons
    val_button = ctl->buttons();
    val_button_A = val_button & (1 << BUTTON_A);
    val_button_B = val_button & (1 << BUTTON_B);
    val_button_X = val_button & (1 << BUTTON_X);
    val_button_Y = val_button & (1 << BUTTON_Y);
    val_button_LB = val_button & (1 << BUTTON_LB);
    val_button_RB = val_button & (1 << BUTTON_RB);
    val_button_STICK_L = val_button & (1 << BUTTON_STICK_L);
    val_button_STICK_R = val_button & (1 << BUTTON_STICK_R);

    // retire la zone morte
    if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
        // code for when left joystick is at idle
        dir = 0;
        vit_cons = 0;
    }

    dir = (-ctl->axisX() / 512.0) * 0.05;
    vit_cons = (-ctl->axisY() / 512.0) * 0.12;
    switch (etat_ctl) {
        case 0:
            kp_theta = 0, kd_theta = 0, kp_vit = 0, kd_vit = 0;
            if (val_button_LB)
                etat_ctl = 1;
            break;
        case 1:
            kp_theta = 2.5, kd_theta = 0.052, kp_vit = 0.088, kd_vit = 0.01;
            dir = (-ctl->axisX() / 512.0) * 0.05;
            if (val_button_RB)
                etat_ctl = 0;
            if (val_button_A)
                etat_ctl = 2;
            if (val_button_B)
                etat_ctl = 3;
            break;
        case 2:
            dir = 0.08;
            if (val_button_A)
                etat_ctl = 1;
            if (val_button_RB)
                etat_ctl = 0;
            if (val_button_B)
                etat_ctl = 3;
            break;
        case 3:
            dir = -0.08;
            if (val_button_B)
                etat_ctl = 1;
            if (val_button_RB)
                etat_ctl = 0;
            if (val_button_A)
                etat_ctl = 2;
            break;
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void controle(void* parameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        ControllerPtr ctl;
        mpu.getEvent(&a, &g, &temp);

        // lis les valeurs de la manette
        bool dataUpdated = BP32.update();
        if (dataUpdated)
            processControllers();

        // Calcul avec l'acceleration
        theta_acc = atan2(a.acceleration.y, a.acceleration.x);
        theta_acc_F = A_teta * theta_acc + B_teta * theta_acc_F_p;  // filtrage
        theta_acc_F_p = theta_acc_F;                                // enregistre la valeur précedente

        // Calcul avec la vitesse de rotation
        // theta_w += (Te / 1000) * -g.gyro.z; // axe z qui tourne intégrale de la vitesse de rotation division par 1000
        // pour passer en seconde avec le filtre sert a rien juste pour observer l'intégrale Calcul simplifié
        theta_w_F = (Tau / 1000) * A_teta * -g.gyro.z + B_teta * theta_w_F_p;  // filtre passe bas
        theta_w_F_p = theta_w_F;
        theta_somme = theta_acc_F + theta_w_F;  // somme des deux positions filtrées

        val_vbatt = 3.3 * analogRead(PIN_VBATT) / (4095.0 * GAIN_PONT_DIV);  // lecture de la tension batt

        // lecture des codeurs
        compteur_codeur_D = encoder_D.getCount();
        compteur_codeur_G = encoder_G.getCount();
        // Calcul des vitesses en m/s
        // - pour avoir une vitesse positive quand il avance
        vitesse_D = -(2 * PI * RAYON_ROUE * (compteur_codeur_D - compteur_codeur_D_p) / NB_TICK_TOURCOMPLET) / (Te);
        vitesse_G = -(2 * PI * RAYON_ROUE * (compteur_codeur_G - compteur_codeur_G_p) / NB_TICK_TOURCOMPLET) / (Te);

        // calcul PID vitesse NON FONCTIONNEL
        erreur_vit_D = vit_cons - vitesse_D;
        erreur_vit_G = vit_cons - vitesse_G;

        P_vit_D = kp_vit * erreur_vit_D;
        P_vit_G = kp_vit * erreur_vit_G;

        D_vit_D = kd_vit * (erreur_vit_D - erreur_vit_D_p) / (Te / 1000);
        D_vit_G = kd_vit * (erreur_vit_D - erreur_vit_D_p) / (Te / 1000);

        // Filtrage de la dérivée
        D_vit_D_F = (Tau_D / 1000) * A_vit * D_vit_D + B_vit * D_vit_D_F_p;
        D_vit_G_F = (Tau_D / 1000) * A_vit * D_vit_D + B_vit * D_vit_G_F_p;
        D_vit_D_F_p = D_vit_D_F;
        D_vit_G_F_p = D_vit_G_F;

        ec_vitt_D = P_vit_D + D_vit_D_F;
        ec_vitt_G = P_vit_G + D_vit_G_F;

        ec_vitt_cons = (ec_vitt_D + ec_vitt_G) / 2;

        ec_vitt_cons = saturation_ec_vit(ec_vitt_cons);
        theta_cons = ec_vitt_cons;

        // Asservissement de position
        erreur_theta = theta_cons - theta_somme + theta0;
        // Calcul de l'intégrale
        Integrale_erreur_theta += (erreur_theta - erreur_theta_p) * Te / 1000;
        // Calcul des composantes PID theta
        P_theta = kp_theta * erreur_theta;
        D_theta = kd_theta * (g.gyro.z);  // dérivé de l'erreur égale à la vitesse angulaire
        I_theta = ki_theta * Integrale_erreur_theta;
        erreur_theta_p = erreur_theta;
        // Calcul de la commande
        ec_theta = P_theta + D_theta + I_theta;

        // sature les commandes + offset pour retirer la zone morte des moteurs [-0.2, 0.2]
        ec_mot_D = saturation_ec_mot(-(ec_theta - dir));
        ec_mot_G = saturation_ec_mot(-(ec_theta + dir));

        // ecriture des PWM
        // analogWrite(PIN_PWM_A_G, 1023 * (0.5 + ec_mot_G  ));
        // analogWrite(PIN_PWM_B_G, 1023 * (0.5 - ec_mot_G  ));
        // analogWrite(PIN_PWM_A_D, 1023 * (0.5 + ec_mot_D  ));
        // analogWrite(PIN_PWM_B_D, 1023 * (0.5 - ec_mot_D  ));
        ledcWriteChannel(CANAL_PWM_A_G, (int)(1023 * (0.5 + ec_mot_G)));
        ledcWriteChannel(CANAL_PWM_B_G, (int)(1023 * (0.5 - ec_mot_G)));
        ledcWriteChannel(CANAL_PWM_A_D, (int)(1023 * (0.5 + ec_mot_D)));
        ledcWriteChannel(CANAL_PWM_B_D, (int)(1023 * (0.5 - ec_mot_D)));

        compteur_codeur_D_p = compteur_codeur_D;
        compteur_codeur_G_p = compteur_codeur_G;

        FlagCalcul = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    kp_theta = 0, kd_theta = 0, kp_vit = 0, kd_vit = 0, theta0 = 0.02, Tau_D = 400;

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // setup MPU6050
    mpu.begin();
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    // setup encoder_D
    // ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder_D.attachHalfQuad(PIN_CODEUR_A_D, PIN_CODEUR_B_D);
    encoder_G.attachHalfQuad(PIN_CODEUR_A_G, PIN_CODEUR_B_G);
    encoder_D.clearCount();  // Reset des compteurs
    encoder_G.clearCount();
    // calcul coeff filtre complémentaire
    A_teta = 1 / (1 + Tau / Te);
    B_teta = Tau / Te * A_teta;

    // calcul coeff vitesse
    A_vit = 1 / (1 + Tau / Te);
    B_vit = Tau / Te * A_vit;

    // setup PWM
    // ledcChangeFrequency(PIN_PWM_A_D, FREQUENCE, RESOLUTION);
    // ledcChangeFrequency(PIN_PWM_B_D, FREQUENCE, RESOLUTION);
    // ledcChangeFrequency(PIN_PWM_A_G, FREQUENCE, RESOLUTION);
    // ledcChangeFrequency(PIN_PWM_B_G, FREQUENCE, RESOLUTION);
    ledcAttachChannel(PIN_PWM_A_D, FREQUENCE, RESOLUTION, CANAL_PWM_A_D);
    ledcAttachChannel(PIN_PWM_B_D, FREQUENCE, RESOLUTION, CANAL_PWM_B_D);
    ledcAttachChannel(PIN_PWM_A_G, FREQUENCE, RESOLUTION, CANAL_PWM_A_G);
    ledcAttachChannel(PIN_PWM_B_G, FREQUENCE, RESOLUTION, CANAL_PWM_B_G);

    pinMode(PIN_LED, OUTPUT);

    xTaskCreate(controle,    // nom de la fonction
                "controle",  // nom de la tache que nous venons de créer
                10000,       // taille de la pile en octet
                NULL,        // parametre
                10,          // tres haut niveau de priorite
                NULL         // descripteur
    );
}

void reception(char ch) {
    static int i = 0;
    static String chaine = "";
    String commande;
    String valeur;
    int index, length;

    if ((ch == 13) or (ch == 10)) {
        index = chaine.indexOf(' ');
        length = chaine.length();
        if (index == -1) {
            commande = chaine;
            valeur = "";
        } else {
            commande = chaine.substring(0, index);
            valeur = chaine.substring(index + 1, length);
        }

        if (commande == "Tau") {
            Tau = valeur.toFloat();
            A_vit = 1 / (1 + Tau / Te);
            B_vit = Tau / Te * A_vit;
            A_teta = 1 / (1 + Tau / Te);
            B_teta = Tau / Te * A_teta;
        }
        if (commande == "Te") {
            Te = valeur.toInt();
            A_vit = 1 / (1 + Tau / Te);
            B_vit = Tau / Te * A_vit;
            A_teta = 1 / (1 + Tau / Te);
            B_teta = Tau / Te * A_teta;
        }
        if (commande == "ec") {
            ec_mot_D = valeur.toFloat();
            ec_mot_G = valeur.toFloat();
        }
        if (commande == "kp_theta") {
            kp_theta = valeur.toFloat();
        }
        if (commande == "kd_theta") {
            kd_theta = valeur.toFloat();
        }
        if (commande == "ki_theta") {
            ki_theta = valeur.toFloat();
        }

        if (commande == "kp_vit") {
            kp_vit = valeur.toFloat();
        }
        if (commande == "kd_vit") {
            kd_vit = valeur.toFloat();
        }
        if (commande == "ki_vit") {
            ki_vit = valeur.toFloat();
        }
        if (commande == "vit_cons") {
            vit_cons = valeur.toFloat();
        }
        if (commande == "theta0") {
            theta0 = valeur.toFloat();
        }
        if (commande == "Tau_D") {
            Tau_D = valeur.toFloat();
        }
        if (commande == "vit_D_cons") {
            vit_D_cons = valeur.toFloat();
        }
        if (commande == "vit_G_cons") {
            vit_G_cons = valeur.toFloat();
        }

        chaine = "";
    } else {
        chaine += ch;
    }
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    if (FlagCalcul == 1) {
        // gestion LED

        switch (etat_led) {
            case 0:
                digitalWrite(PIN_LED, HIGH);
                if ((vit_cons > 0.025) || (vit_cons < -0.025))
                    etat_led = 1;
                if (val_vbatt < 9.3)
                    etat_led = 2;
                break;
            case 1:
                digitalWrite(PIN_LED, LOW);
                if ((vit_cons < 0.025) || (vit_cons > -0.025))
                    etat_led = 0;
                if (val_vbatt < 9.3)
                    etat_led = 2;
                break;
            case 2:
                digitalWrite(PIN_LED, LOW);
                vTaskDelay(200);
                digitalWrite(PIN_LED, HIGH);
                digitalWrite(PIN_LED, LOW);
                if ((vit_cons < 0.025) || (vit_cons > -0.025))
                    etat_led = 0;
                if ((vit_cons > 0.025) || (vit_cons < -0.025))
                    etat_led = 1;
                break;
        }

        Serial.printf("%3.5f LB:%x  RB:%x  A:%x  B:%x  %d  %d\n", val_vbatt, val_button_LB, val_button_RB, val_button_A,
                      val_button_B, etat_ctl, etat_led);
        FlagCalcul = 0;
    }
}

void serialEvent() {
    while (Serial.available() > 0)  // tant qu'il y a des caractères à lire
    {
        reception(Serial.read());
    }
}

float saturation_ec_mot(float ec_mot) {
    // Sature entre -0.45 et +0.45 soit un rapport cyclique entre 5-95 %
    // reduit la zone morte
    if (ec_mot > 0)
        ec_mot += 0.18;
    if (ec_mot < 0)
        ec_mot -= 0.18;

    // saturation
    if (ec_mot > 0.45)
        ec_mot = 0.45;
    if (ec_mot < -0.45)
        ec_mot = -0.45;

    return ec_mot;
}

float saturation_ec_vit(float ec_vit) {
    // saturation
    if (ec_vit > 0.05)
        ec_vit = 0.05;
    if (ec_vit < -0.05)
        ec_vit = -0.05;

    return ec_vit;
}