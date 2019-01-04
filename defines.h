#ifndef DEFINE_H
#define DEFINE_H

#define PLAN_B
//#define OUT_USB


enum ID
{
    IDO_MG,
    IDO_MH,
    IDO_MD,
    IDO_MB,
    IDO_M1,
    IDO_M2,
    IDO_M3,
    IDO_M4,
    IDO_M5,
    IDO_M6,
    IDO_D1,
    IDO_D2,
    IDO_D3,
    IDO_D4,
    IDO_E,
    IDO_S,
    IDO_PC1,
    IDO_PC2,
    IDO_PC3,
    IDO_PC4,
    IDO_PC5,
    IDO_P1,
    IDO_P2,
    IDO_P3,
    IDO_P4,
    IDO_P5,
    IDO_P6,
    IDO_P7,
    IDO_P8,
    IDO_P9,
    IDO_P10,
    IDO_P11,
    IDO_P12,
    IDO_P13,
    IDO_P14,
    IDO_P15,
    IDO_P16,
    IDO_DEPOT_PC,
    IDO_DEPOT_P
};

#define ROBOTRADIUS 190

#define MAXPOINT 8000

// ----- Loggeur ----- //

#ifdef OUT_USB
    #define OUT_TX USBTX
    #define OUT_RX USBRX
#else
    #define OUT_TX PA_11
    #define OUT_RX PA_12
#endif

// ----- Moteurs ----- //

#define PWM_MOT1 PB_13
#define PWM_MOT2 PB_14
#define PWM_MOT3 PB_15

#define DIR_MOT1 PC_9
#define DIR_MOT2 PB_8
#define DIR_MOT3 PB_9

// ----- Odometrie ----- //

#define ODO_G_B PA_10
#define ODO_G_A PB_3

#define ODO_D_B PB_5
#define ODO_D_A PB_4

#define PI 3.14159f

// ----- Boutons ----- //

#define LED_DESSUS PH_1
#define BP_DESSUS PC_8
#define TIRETTE_DESSUS PC_6
#define COULEUR_DESSUS PC_5

#define COULEUR_JAUNE 0
#define COULEUR_VERTE 1

// ----- AX12 ----- //

#define AX12_TX PA_9
#define AX12_RX NC

#define MAX_TORQUE 300

#define BRASG_OUVERT 60
#define BRASG_FERME 155
#define BRASD_OUVERT 240
#define BRASD_FERME 145

#define PINCE_OUVERTE 100
#define PINCE_FERMEE 3

// ----- Sharp ----- //

#define SHARP_D A4
#define SHARP_DG A3
#define SHARP_DD A5
#define SHARP_AG A2
#define SHARP_AD A1

#endif
