
/***********************************************
 * Jarrett Cigainero, April 2018
 * Basic CNC
***********************************************/
#include <p30f3011.h>
#include <dsp.h>
//#include "main.h"

//Time critical calulation for integral math.
// Tm = 32767 * (1 / (((clkSpeedmhz * PLL) / 4) / tiksPerIRQ))
#define clkSpeedmhz 10;
#define IPS 29.48;   //million
#define BAUD 9600;
#define tiksPerIRQ 610;
#define intTime 1;
/******************************************/
/* Stepper Direction Inversion. */
#define Xinvrt 0;
#define Yinvrt 1;
#define Zinvrt 0;
#define Ainvrt 0;
/******************************************/
/* Resolution and duty cycle*/
#define reso 250;
#define DTcycl 100;
/******************************************/
#define cmdBuf 200;
/******************************
* Chip Configuration
******************************/

_FOSC(CSW_FSCM_OFF & FRC_PLL16);   /* osc with PLLx8 */
_FWDT(WDT_OFF);                 /* Watch dog Timer OFF */
_FBORPOR(PBOR_OFF & MCLR_EN);   /* Brown Out OFF, MCLR Enabled */
_FGS(CODE_PROT_OFF);            /* Code Protect OFF */

/***********************************************************
 * Variables 
***********************************************************/
void stepVarInit (void);
void StepperGo(void);
unsigned int BaudCalc(float, float);
void kyScan(void);

/*****************************/
/* Init vars and stuff. */
/*****************************/
    int ToolTimeOut = 5;    //5 seconds.
    int TempTimeOut = 5;
    int TimeTgl = 0;
    int TimeStop = 0;
    int Done = 1;
    int Stop = 0;
    int KeyGO = 0;
    int KTGL = 0;
    int SftStart = 0;
    int Pdelay = 0;
    unsigned char KeyScan = 0;
    int ToolCMP = 1;
    int pRdy = 0;
    int HTorder = 1;
    int SoftStart = 0;
    int varPR = 0;
    int prPrep = 0;
    long spdMaster = 0;
    int mSpd = 0;
    int Lecho = 0;
    int maxSpeed = 99;
    int stepGO = 0;
    int Trdy = 0;
    int TrdyTemp = 0;
    int OutBool = 0;
    int Terr = 0;
    int ToolRDY = 0;
    int ToolConfig = 0;
    int toolRCount = 0;
    int ToolPower = 0;
    int ToolPowertmp = 0;
    int bufsize = 0;
    int Xivrt = Xinvrt;
    int Yivrt = Yinvrt;
    int Zivrt = Zinvrt;
    int Aivrt = Ainvrt;
    int DirOut = 0;
    int PulseOut = 0;
    int Xclc = 0;
    int Yclc = 0;
    int Zclc = 0;
    int Aclc = 0;
    int Xpls = 0;
    int Ypls = 0;
    int Zpls = 0;
    int Apls = 0;
    long Xdiv = 0;
    long Ydiv = 0;
    long Zdiv = 0;
    long Adiv = 0;
    int Xcmd = 0;
    int Ycmd = 0;
    int Zcmd = 0;
    int Acmd = 0;
    long Xpos = 0;
    long Ypos = 0;
    long Zpos = 0;
    long Apos = 0;
    long PosMaster = 0;
    float Xrate = 0;
    float Yrate = 0;
    float Zrate = 0;
    float Arate = 0;
/*************************/
    /* Command vars and stuff. */
    static int cdmd = cmdBuf;
    char cmd[200];
    unsigned int cmdPoint = 0;
    int cmdRDY = 0;
    int cRDY = 0;
    int cmdOVFL = 0;
    void reInit(void);


/***********************************************************
***********************************************************/
int main(void)
{

/*******************************
 * initialization values setup.
*******************************/
    /* Used as stepper Output */
    TRISB = 0;              //set portb to all outputs
    LATB = 0;               //clear portb
    /**************************/
    /* PWM outputs. */
    TRISE = 0xFFFE;              //set porte to all inputs and one Output.
    //LATE = 0;               //set all porte outputs to 0
    /**************************/
    /* Key Pad output control */
    /* RF0, RF1, RF6 */
    /* Pins 27 and 28 should be reserved for AUX com on UART2. */
    TRISF = 0;              //set portf to all outputs
    LATF = 0;               //set all portf outputs to 0
    /**************************/
    /* RD0, RD2, RD3 are for KeyPad inputs. */
    TRISD = 0xFFFF;              //set portd to all inputs
    //LATD = 0;               //set all portd outputs to 0
    /**************************/
    /* Status Light */
    TRISC = 0;
    LATC = 0;

/*****************************/
/* Configure PWM */
/*****************************/
    PTCON = 0x0006;     //disable the PWM module and set to up/down mode for center aligned PWM.
    PTPER = 0x63;         //set period. 0% - 99%
    PWMCON1 = 0x00FF;       //Set PWM output for complementary mode.
    PDC1 = 0000;            //set output to 0
    PDC2 = 0000;            //set output to 0
    PDC3 = 0000;            //set output to 0

/*****************************/
/* Configure UARTs */
/*****************************/
    U1STA = 0;
    U1BRG = BaudCalc(9600, 30);     //9600 baud at 30 mips.
    U1MODEbits.ALTIO = 1;           //Use alternate IO.
    //Default power up of UART should be 8n1
/*****************************/
/* Configure Timer 1 */
/*****************************/
/* for general purpose and Stepper Motor Control. */
    PR1 = 0x0FFF;
    TMR1 = 0x0000;
    T1CON = 0x0000;
    
/*****************************/
/* Configure Timer 2 */
/*****************************/
/* For half second timing operations. */
    PR2 = 0xE4E2;   //58,594
    TMR2 = 0x0000;
    T2CON = 0x0000;
    T2CONbits.TCKPS = 3;

/*****************************/
/* Disable analog inputs */
/*****************************/
    
    ADPCFG = 0xFFFF;

/*****************************/
/* Configure IRQs. */
/*****************************/
    //Configure Priorities
    IPC2bits.ADIP = 1;
    IPC0bits.T1IP = 3;
    IPC1bits.T2IP = 4;
    IPC2bits.U1RXIP = 5;
    IPC0bits.INT0IP = 6;
    IPC5bits.INT2IP = 7;
        	// Clear all interrupts flags
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
	IFS0bits.T1IF = 0;	// Clear timer 1 flag
    IFS0bits.T2IF = 0;	// Clear timer 2 flag
    IFS0bits.INT0IF = 0;
    IFS1bits.INT2IF = 0;
    IFS0bits.ADIF = 0;  // Clear ADC IRQ flag
    IFS0bits.U1RXIF = 0; //Clear UART1 flag

	// enable all interrupts
	__asm__ volatile ("DISI #0x3FFF");
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
	IEC0bits.T1IE = 1;	// Enable interrupts for timer 1
    IEC0bits.U1RXIE = 1; //Enable interrupts for UART1 Rx.
    IEC0bits.INT0IE = 1;
    IEC1bits.INT2IE = 1;
    IEC0bits.T2IE = 1;	// Enable interrupts for timer 2
    //IEC0bits.ADIE = 1;  // Enable ADC IRQs.
    INTCON2bits.INT0EP = 0;
    INTCON2bits.INT2EP = 0;
DISICNT = 0;
/*****************************/
/* Enable some of our devices. */
/*****************************/
    //ADCON1bits.ADON = 1;    // turn ADC ON 
    PTCONbits.PTEN = 1;     // Enable PWM
    T2CONbits.TON = 1;      // Start Timer 2
    T1CONbits.TON = 1;      // Start Timer 1
    U1MODEbits.UARTEN = 1;  //enable UART 
    U1STAbits.UTXEN = 1;

/*****************************/
    // HTorder
/*****************************/
    TempTimeOut = ToolTimeOut;
    for (;;)                    //loop forever.
    {
        if (cRDY == 1 && Stop <= 0 && TimeStop <= 0){
            Done = 0;
            reInit();
            cRDY = 0;
            ToolRDY = 0;
            toolRCount = ToolConfig;
            if (HTorder == 1){
                PDC1 = ToolPower;
                while (ToolRDY == 0 && Trdy == 1){  //If one becomes false it exits.
                    Idle();
                }
                StepperGo();
                while (stepGO == 1){  //Wait for both head and tool to finish.
                    Idle();
                }
            }
            else {
                StepperGo();
                while (stepGO == 1){  //Continue when Head Movement is done.
                    Idle();
                }
                PDC1 = ToolPower;
                while (ToolRDY == 0 && Trdy == 1){  //Wait for tool to finish.
                    Idle();
                }
            }
            Done = 1;
            while (Stop == 1 || TimeStop == 1){
                Idle();
            }
            if (Stop <= 0 && TimeStop <= 0){
                U1TXREG = 0x63;   //Send command completed to host. "c"
            }
        }
        if (KeyScan > 1 && KeyGO == 1 && Stop == 1){
            KeyGO = 0;
            StepperGo();
            while (stepGO == 1){  //Continue when Head Movement is done.
                Idle();
            }
        }
        Idle();     //Idle Loop, saves power.
    }
    return 0;
}

/********************************************
 ********************************************
 ********************************************
 ********************************************/
void StepperGo(void){
     /* Check for direction Inversion. 1 means invert. */
    if (Xivrt == 1)
        Xcmd *= -1;
    if (Yivrt == 1)
        Ycmd *= -1;
    if (Zivrt == 1)
        Zcmd *= -1;
    if (Aivrt == 1)
        Acmd *= -1;
    /* Calculate direction Outputs. 1 = +, 0 = - */
    DirOut = 0;
    if (Xcmd >= 0)
        DirOut |= 1;
    if (Ycmd >= 0)
        DirOut |= 2;
    if (Zcmd >= 0)
        DirOut |= 4;
    if (Acmd >= 0)
        DirOut |= 8;
    /* Enable the stepper drivers */
    DirOut |= 256;
    /* Command now needs to be absolute. */
    if (Xcmd < 0)
        Xcmd *= -1;
    if (Ycmd < 0)
        Ycmd *= -1;
    if (Zcmd < 0)
        Zcmd *= -1;
    if (Acmd < 0)
        Acmd *= -1;
    /* Get the largest movement to use for PosMaster. */
    if (Xcmd > Ycmd){
        if (Xcmd > Zcmd){
            if (Xcmd > Acmd){
                PosMaster = Xcmd;
            }
            else {
                PosMaster = Acmd;
            }
        }
        else{
            if (Zcmd > Acmd){
                PosMaster = Zcmd;
            }
            else {
                PosMaster = Acmd;
            }
        }
    }
    else{
        if (Ycmd > Zcmd){
            if (Ycmd > Acmd){
                PosMaster = Ycmd;
            }
            else {
                PosMaster = Acmd;
            }
        }
        else{
            if (Zcmd > Acmd){
                PosMaster = Zcmd;
            }
            else {
                PosMaster = Acmd;
            }
        }
    }
    /**************************************************/
    /* Calculate inverse slopes relative to PosMaster. */
    float FPosMaster;
    float FXcmd;
    float FYcmd;
    float FZcmd;
    float FAcmd;
    FPosMaster = PosMaster;
    FXcmd = Xcmd;
    FYcmd = Ycmd;
    FZcmd = Zcmd;
    FAcmd = Acmd;
    if (Xcmd != 0)
        Xrate = FPosMaster / FXcmd;
    if (Ycmd != 0)
        Yrate = FPosMaster / FYcmd;
    if (Zcmd != 0)
        Zrate = FPosMaster / FZcmd;
    if (Acmd != 0)
        Arate = FPosMaster / FAcmd;
    /* Calculate the timing dividers. */
    float rezz;
    rezz = reso;
    Xdiv = rezz * Xrate;
    Ydiv = rezz * Yrate;
    Zdiv = rezz * Zrate;
    Adiv = rezz * Arate;
    PosMaster *= reso;
    /* Calculate the max speed var */
    mSpd = maxSpeed * -1;
    mSpd += 100;
    mSpd *= 10;
    if (mSpd > 1000){
        mSpd = 1000;
    }
    if (SftStart){
        spdMaster = reso;
        PR1 = 1000;   //Head Start Speed
        prPrep = 1000;
        varPR = 80;
    }
    else {
        PR1 = mSpd;
    }
    /********************************/
    /* Here we go! */
    stepGO = 1;     //Vars are prepared, now go!
}

/* Tool Ready IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt (void){
    if(ToolCMP == 1){
        ToolRDY = 1;
    }

    IFS0bits.INT0IF = 0;
}

/* Tool Rotate IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt (void){
    toolRCount--;
    TempTimeOut = ToolTimeOut;      //Reset Tool Timer
    if(ToolCMP == 0){
        ToolRDY = 1;
    }
    if (toolRCount < 1 && ToolConfig > 0){
        PDC1 = 0;
    }
    IFS1bits.INT2IF = 0;
}

/* Data and Command input and processing. */
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void){
    int Tneg = 1;
    int T1 = 0;
    int T2 = 0;
    int T3 = 0;
    int T4 = 0;
    
    while (U1STAbits.URXDA && cmdRDY == 0){       //Data ready?
        cmd[cmdPoint] = U1RXREG;
        if (Lecho){
            U1TXREG = cmd[cmdPoint];
        }
        if (cmd[cmdPoint] == 0x0D){     //Check for a RETURN
            if (Lecho){
                U1TXREG = 0x0A;             //Send NewLine
            }
            bufsize = cmdPoint;
            cmdPoint = 0;
            cmdRDY = 1;                 //Tell our command handler to process.
        }
        else if (cmdPoint > cdmd){      //Check for buffer overflow.
            cmdPoint = 0;
            cmdOVFL = 1;
        }
        else {
            cmdPoint++;
        }
    }
    if (cmdRDY == 1){
        int tempPoint = 0;
        U1TXREG = 0x61;                 //Send command receive "a"
        
        while (tempPoint < bufsize){
            if (cmd[tempPoint] == 0x0D){    //Enter Key.
                if (Lecho){
                    U1TXREG = 0x0A;             //Send NewLine
                }
                break;                      //Used as a test, do nothing.
            }
            else if (cmd[tempPoint] == 0x23){   // "#" Reset.
                PDC1 = 0000;
                cmdPoint = 0;
                cmdRDY = 0;
                asm("reset");
            }
            else if (cmd[tempPoint] == 0x54){   //Tool movement config. "T"
                tempPoint++;
                ToolConfig = cmd[tempPoint];
                ToolConfig -= 48;
                if (ToolConfig < 0 || ToolConfig > 9){
                    Terr = -1;            //Tool config error.
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x45){   //Local Echo config. "E"
                tempPoint++;
                if (cmd[tempPoint] == 0x79){
                    Lecho = 1;
                }
                else if (cmd[tempPoint] == 0x6E){
                    Lecho = 0;
                }
                else {
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x52){   //Tool Ready config. "R"
                tempPoint++;
                cRDY = 1;
                if (cmd[tempPoint] == 0x79){    //"y" Wait until Tool is ready.
                    TrdyTemp = 1;
                }
                else if (cmd[tempPoint] == 0x6E){   //"n" Don't wait.
                    TrdyTemp = 0;
                }
                else {
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x53){    //Head Speed Ramp. "S"
                tempPoint++;
                if (cmd[tempPoint] == 0x79){    //"y"
                    SoftStart = 1;
                }
                else if (cmd[tempPoint] == 0x6E){   //"n"
                    SoftStart = 0;
                }
                else {
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x4D){    //MaxSpeed. "M"
                tempPoint++;
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 10;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                maxSpeed = T1 + T2;
                if (maxSpeed <= 0){
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x50){    //Tool Power. "P"
                tempPoint++;
                cRDY = 1;
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 10;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                ToolPowertmp = T1 + T2;
                if (maxSpeed <= 0){
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x4F){    //Tool/Head, what starts first. Order. "O"
                tempPoint++;
                if (cmd[tempPoint] == 0x74){    // "t"
                    HTorder = 1;
                }
                else if (cmd[tempPoint] == 0x68){   // "h"
                    HTorder = 0;
                }
                else {
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x43){    //Tool complete. when is the tool finished?. "C"
                tempPoint++;
                if (cmd[tempPoint] == 0x72){    // "r"  Tool Ready.
                    ToolCMP = 1;
                }
                else if (cmd[tempPoint] == 0x63){   // "c"   Tool Rotate.
                    ToolCMP = 0;
                }
                else {
                    Terr = -1;
                }
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x58){    //Xdata.
                cRDY = 1;
                tempPoint++;
                Tneg = 1;
                if (cmd[tempPoint] == 0x2D){    //look for negative numbers.
                    Tneg = -1;
                    tempPoint++;
                }
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 1000;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                T2 *= 100;
                tempPoint++;
                T3 = cmd[tempPoint] - 48;
                if (T3 < 0 || T3 > 9){Terr = -1;}
                T3 *= 10;
                tempPoint++;
                T4 = cmd[tempPoint] - 48;
                if (T4 < 0 || T4 > 9){Terr = -1;}
                Xcmd = T1 + T2 + T3 + T4;
                Xcmd *= Tneg;
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x59){    //Ydata.
                cRDY = 1;
                tempPoint++;
                Tneg = 1;
                if (cmd[tempPoint] == 0x2D){    //look for negative numbers.
                    Tneg = -1;
                    tempPoint++;
                }
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 1000;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                T2 *= 100;
                tempPoint++;
                T3 = cmd[tempPoint] - 48;
                if (T3 < 0 || T3 > 9){Terr = -1;}
                T3 *= 10;
                tempPoint++;
                T4 = cmd[tempPoint] - 48;
                if (T4 < 0 || T4 > 9){Terr = -1;}
                Ycmd = T1 + T2 + T3 + T4;
                Ycmd *= Tneg;
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x5A){    //Zdata.
                cRDY = 1;
                tempPoint++;
                Tneg = 1;
                if (cmd[tempPoint] == 0x2D){    //look for negative numbers.
                    Tneg = -1;
                    tempPoint++;
                }
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 1000;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                T2 *= 100;
                tempPoint++;
                T3 = cmd[tempPoint] - 48;
                if (T3 < 0 || T3 > 9){Terr = -1;}
                T3 *= 10;
                tempPoint++;
                T4 = cmd[tempPoint] - 48;
                if (T4 < 0 || T4 > 9){Terr = -1;}
                Zcmd = T1 + T2 + T3 + T4;
                Zcmd *= Tneg;
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x41){    //Adata.
                cRDY = 1;
                tempPoint++;
                Tneg = 1;
                if (cmd[tempPoint] == 0x2D){    //look for negative numbers.
                    Tneg = -1;
                    tempPoint++;
                }
                T1 = cmd[tempPoint] - 48;
                if (T1 < 0 || T1 > 9){Terr = -1;}
                T1 *= 1000;
                tempPoint++;
                T2 = cmd[tempPoint] - 48;
                if (T2 < 0 || T2 > 9){Terr = -1;}
                T2 *= 100;
                tempPoint++;
                T3 = cmd[tempPoint] - 48;
                if (T3 < 0 || T3 > 9){Terr = -1;}
                T3 *= 10;
                tempPoint++;
                T4 = cmd[tempPoint] - 48;
                if (T4 < 0 || T4 > 9){Terr = -1;}
                Acmd = T1 + T2 + T3 + T4;
                Acmd *= Tneg;
                tempPoint++;
            }
            else if (cmd[tempPoint] == 0x20){           // Space.
                tempPoint++;
            }
            else {
                U1TXREG = 0x53;     //Print "S" for Syntax Error.
                Terr = -1;
                break;
            }
        }
        reInit();
        TempTimeOut = ToolTimeOut;
        cmdPoint = 0;
        cmdRDY = 0;
        if (Terr == -1){
            PDC1 = 0;
            cRDY = 0;
            Terr = 0;
            cmdPoint = 0;
            tempPoint = 0;
            U1TXREG = 0x45;     //Print "E" for Error.
            Xcmd = 0;
            Ycmd = 0;
            Zcmd = 0;
            Acmd = 0;
            ToolPowertmp = 0;
            ToolPower = 0;
        }
    }
    IFS0bits.U1RXIF = 0;
}



/* Stepper Motor Timed Output. */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void){
    
    kyScan();   //Scan Keypad.
/*************************************/
/* Stepper Motor Key Run */
    /* Soft Start on Key Press */
    if (Stop == 1 && Done == 1){
        PR1 = mSpd;
        SftStart = 0;
        KTGL = 1;
        Trdy = 0;
        ToolPower = 0;
    }
    /****************************/
    if ((KeyScan == 2 || KeyScan == 3) && Stop == 1 && Done == 1){
        KeyGO = 1;
        if (KeyScan == 2)
            Xcmd = 10;
        else
            Xcmd = -10;
    }
    else if ((KeyScan == 5 || KeyScan == 6) && Stop == 1 && Done == 1){
        KeyGO = 1;
        if (KeyScan == 5)
            Ycmd = 10;
        else
            Ycmd = -10;
    }
    else {
        KeyGO = 0;
    }
/*************************************/

/* Stepper Motor Normal Run */
    if (stepGO >= 1 || KeyGO == 1){
    //do stuff.
        if (Xdiv != 0){
            if (Xpos >= Xdiv){
                PulseOut |= 16;
                Xpos = 1;
                Xpls = 0;
            }
            else {
                Xpos++;
                if (Xpls <= 0){
                    Xpls = 1;
                    Xclc = DTcycl;
                }
            }
        }
        
        if (Ydiv != 0){
            if (Ypos >= Ydiv){
                PulseOut |= 32;
                Ypos = 1;
                Ypls = 0;
            }
            else {
                Ypos++;
                if (Ypls <= 0){
                    Ypls = 1;
                    Yclc = DTcycl;
                }
            }
        }
        
        if (Zdiv != 0){
            if (Zpos >= Zdiv){
                PulseOut |= 64;
                Zpos = 1;
                Zpls = 0;
            }
            else {
                Zpos++;
                if (Zpls <= 0){
                    Zpls = 1;
                    Zclc = DTcycl;
                }
            }
        }
        
        if (Adiv != 0){
            if (Apos >= Adiv){
                PulseOut |= 128;
                Apos = 1;
                Apls = 0;
            }
            else {
                Apos++;
                if (Apls <= 0){
                    Apls = 1;
                    Aclc = DTcycl;
                }
            }
        }
/*********************************/

/* Stepper Motor Duty Cycle */
    if (Xclc > 0){
        Xclc--;
    }
    else if (Xclc == 0){
        Xclc = -1;
        PulseOut &= 0xEF;
    }
    if (Yclc > 0){
        Yclc--;
    }
    else if (Yclc == 0){
        Yclc = -1;
        PulseOut &= 0xDF;
    }
    if (Zclc > 0){
        Zclc--;
    }
    else if (Zclc == 0){
        Zclc = -1;
        PulseOut &= 0xBF;
    }
    if (Aclc > 0){
        Aclc--;
    }
    else if (Aclc == 0){
        Aclc = -1;
        PulseOut &= 0x7F;
    }
/******************************/

/* Stepper Motor Speed Calculation */
        if (PosMaster < 0){
            stepGO = 0;
            DirOut = 0;
            PulseOut = 0;
            stepVarInit();
        }
        else {
            if (SftStart){
                if (spdMaster > 0){
                    spdMaster--;
                }
                else {
                    if (PosMaster < 20000){
                        prPrep += 10;
                        varPR = 80;
                        if (prPrep >= 2000){
                            prPrep = 2000;
                        }
                    }
                    else {
                        prPrep -= varPR;
                        if (varPR > 2 && prPrep < 100){
                            varPR--;
                        }
                        if (prPrep <= mSpd){
                            prPrep = mSpd;
                        }
                    }
                    PR1 = prPrep;
                    spdMaster = reso;
                }
            }
            else {
                PR1 = mSpd;
            }
            PosMaster--;
        }
    }
    else {
        PR1 = 0x0FFF;
    }
/*********************************/

/* Put it all together and send it out. */
    LATB = PulseOut | DirOut;
/****************************************/
    /* End the IRQ. */
	IFS0bits.T1IF = 0;
}
/***********************************************/
/***********************************************/
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void){
    if (TimeTgl == 0){
        TimeTgl = 1;
        LATCbits.LATC15 = 1;
    }
    else {
        if (TimeStop){
            LATCbits.LATC15 = 0;
        }
        TimeTgl = 0;
        if (TempTimeOut <= 0 && Stop == 0){
            TimeStop = 1;
            PDC1 = 0000;
        }
        else if (Stop == 0 && ToolPowertmp > 0) {
            TempTimeOut--;
        }
        else if (Stop == 1 || ToolPowertmp <= 0){
            TimeStop = 0;
            TempTimeOut = ToolTimeOut;
        }
    }
/****************************************/
    /* End the IRQ. */
	IFS0bits.T2IF = 0;   
}

/***********************************************/
/***********************************************/
void reInit(void){
        SftStart = SoftStart;
        ToolPower = ToolPowertmp;
        Trdy = TrdyTemp;
        KTGL = 0;
}


void stepVarInit (void){
    Xdiv = 0;
    Ydiv = 0;
    Zdiv = 0;
    Adiv = 0;
    stepGO = 0;
    DirOut = 0;
    PulseOut = 0;
    Xcmd = 0;
    Ycmd = 0;
    Zcmd = 0;
    Acmd = 0;
    Xpos = 0;
    Ypos = 0;
    Zpos = 0;
    Apos = 0;
    PosMaster = 0;
    Xrate = 0;
    Yrate = 0;
    Zrate = 0;
    Arate = 0;
}

unsigned int BaudCalc(float BD, float mlt){
    /* Calculate baud rate. */
    float INS;
    float OutPut;
    unsigned int Oputs;
    INS = mlt * 1000000;
    OutPut = ((INS/BD)/16)-1;
    Oputs = OutPut;
    return Oputs;             //Weird things happen when you try to calculate
                              //a float directly into an int.
    /************************/
}

void kyScan(void){
    KeyScan = 0;
    LATF = 0;
    LATFbits.LATF0 = 1;
    if (Pdelay == 0){
        Pdelay = 1;
        return;
    }
    Stop = 0;
    if (PORTDbits.RD0 == 1){
        if (Done){
            Stop = 1;           //Used for Stop Switch.
        }
        //KeyScan = 1;      
    }
    if (PORTDbits.RD2 == 1){
        KeyScan = 2;
    }
    if (PORTDbits.RD3 == 1){
        KeyScan = 3;
    }
    LATF = 0;
    LATFbits.LATF1 = 1;
    if (Pdelay == 1){
        Pdelay = 2;
        return;
    }
    if (PORTDbits.RD0 == 1){
        KeyScan = 4;
    }
    if (PORTDbits.RD2 == 1){
        KeyScan = 5;
    }
    if (PORTDbits.RD3 == 1){
        KeyScan = 6;
    }
    LATF = 0;
    LATFbits.LATF6 = 1;
    if (Pdelay == 2){
        Pdelay = 0;
        return;
    }
    if (PORTDbits.RD0 == 1){
        KeyScan = 7;
    }
    if (PORTDbits.RD2 == 1){
        KeyScan = 8;
    }
    if (PORTDbits.RD3 == 1){
        KeyScan = 9;
    }
}

