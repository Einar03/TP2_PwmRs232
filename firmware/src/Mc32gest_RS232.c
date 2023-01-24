// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'émission et de réception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoyé réponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

// Structure décrivant le message
typedef struct {
    uint8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    uint8_t MsbCrc;
    uint8_t LsbCrc;
} StruMess;


// Struct pour émission des messages
StruMess TxMess;
// Struct pour réception des messages
StruMess RxMess;
// Union pour le CRC du message
U_manip16 RxValCrc16;

// Declaration des FIFO pour réception et émission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrFifoRX;

int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrFifoTX;

// Initialisation de la communication sérielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de réception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'émission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit émission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message reçu donc local (data non modifié)
// Valeur de retour 1  = message reçu donc en remote (data mis à jour)
int GetMessage(S_pwmSettings *pData)
{
    // Variables
    int commStatus = 0;
	uint8_t NbCharInFifo = 0;
	uint8_t	findFlag = 0;
	uint8_t i = 0;
	uint16_t ValCrc16 = 0;
    static uint8_t NbCharToRead = 5;
	static uint8_t startIndex = 0;
    static int8_t tb_ReadValues[5] = {0,0,0,0,0};
	
    // Traitement de réception à introduire ICI
    // Lecture et décodage fifo réception
    // =====================================================================
    // Récupération du nombre de bytes dans le fifo
    NbCharInFifo = GetReadSize(&descrFifoRX);
    // Si 5 bytes (message complet) dans le fifo, on analyse le message
    if(NbCharInFifo >= NbCharToRead)
    {
<<<<<<< HEAD
		// Récupération et sauvegarde des 5 bytes du message 
        // complet dans tb_ReadValues
		for(i = 0; i < (NbCharToRead - 1); i++)
        commStatus = GetCharFromFifo(&descrFifoRX, 
                &tb_ReadValues[i + 5 - startIndex]);
		if(NbCharToRead != 5)
		{
			NbCharToRead = 5;
			startIndex = 0;
		}
		// Si le premier byte est 0xAA (-86 avec int8_t => STX_code)
		if(tb_ReadValues[0] == STX_code)
		{
			// Récupération et sauvegarde des données CRC réçues
            // (MSL et LSB) dans RxValCrc16
			RxValCrc16.shl.msb = tb_ReadValues[3];
			RxValCrc16.shl.lsb = tb_ReadValues[4];
			ValCrc16 = 0xFFFF;
            ValCrc16 = updateCRC16(ValCrc16, tb_ReadValues[0]);
            ValCrc16 = updateCRC16(ValCrc16, tb_ReadValues[1]);
            ValCrc16 = updateCRC16(ValCrc16, tb_ReadValues[2]);
            
			// Si le CRC calculé égal CRC du message
			if(ValCrc16 == RxValCrc16)
			{
				// Mettre à jour les variables "Speed" et "Angle"
				pData -> SpeedSetting = tb_ReadValues[1];
				pData -> AngleSetting = tb_ReadValues[2];
				commStatus = 1;
			}
			else
			{
				commStatus = 0;
			}
		}
		else
		{
			//i = 0;
			// Recherche du début du message 0xAA dans les données reçues
			for(i = 1; i < (NbCharToRead - 1); i++)
			{
				if(findFlag != 0)
				{
					if(startIndex < 4)
					{
						// Sauvegarde du reste des données à la suite
                        // de la première case du tableau
						tb_ReadValues[i-startIndex] = tb_ReadValues[i];
					}
				}
				else
				{
					// 
					if(tb_ReadValues[i] == STX_code)
					{
						// Activer le flag
						findFlag = 1;
						// Récupération de l'indice du début du message
						startIndex = i;
						// Deplacement du byte de start dans la premier case
                        // de tb_ReadValues
						tb_ReadValues[0] = tb_ReadValues[i];
						// Calcul du nombre de bytes qui restent pour 
                        // le message complet est sauvegarde dans
						NbCharToRead = startIndex;
						
					}
				}
			}
		}
=======
        GetCharFromFifo();
>>>>>>> origin/Jonathan
    }
    
    // Gestion controle de flux de la réception
    if(GetWriteSpace ( &descrFifoRX) >= (2*MESS_SIZE)) {
        // autorise émission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage


// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
    int8_t FreeSize;
    //selon spec. CCITT il faut initialiser la valeur du Crc16 à 0xFFFF
    int8_t ValueCrc16 = 0xFFFF; 

    // Traitement émission à introduire ICI
    // Formatage message et remplissage fifo émission
    // ...
     FreeSize = GetWriteSpace (&descrFifoTX);
    if (FreeSize >= MESS_SIZE )
    {
      // Composition du message
         //message de start = STX_code (0xAA)
        TxMess.Start = STX_code;
        //prépare le message de l'angle
        TxMess.Angle = pData->AngleSetting;
        //prépare le message de la vitesse
        TxMess.Speed = pData->SpeedSetting;
        
        //Calcul du CRC sur 3 premiers valeurs
        ValueCrc16 = updateCRC16 (ValueCrc16, TxMess.Start);
        ValueCrc16 = updateCRC16 (ValueCrc16, TxMess.Speed);
        ValueCrc16 = updateCRC16 (ValueCrc16, TxMess.Angle);  
        
        //recupeler valeurs du message
        TxMess.MsbCrc = (ValueCrc16 & 0xFF00) >> 8;  // récupère MSB du CRC par masquage
        TxMess.LsbCrc = (ValueCrc16 & 0x00FF);  // récupère LSB du CRC par masquage
       
        // Dépose le message dans le fifo
        PutCharInFifo (&descrFifoTX, TxMess.Start);
        PutCharInFifo (&descrFifoTX, TxMess.Speed);
        PutCharInFifo (&descrFifoTX, TxMess.Angle);
        PutCharInFifo (&descrFifoTX, TxMess.MsbCrc);
        PutCharInFifo (&descrFifoTX, TxMess.LsbCrc);  
    }
    // Gestion du controle de flux
    // si on a un caractère à envoyer et que CTS = 0
    FreeSize = GetReadSize(&descrFifoTX);
    if ((RS232_CTS == 0) && (FreeSize > 0))
    {
        // Autorise int émission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}

// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la réponse générée dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)    
{
    
     uint8_t freeSize, TXSize;
     int8_t c;
     int8_t i_cts = 0;
     BOOL TxBuffFull;
     
     USART_ERROR UsartStatus;    


    // Marque début interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur à la réception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {

        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            // Traitement RX à faire ICI
            // Lecture des caractères depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                c = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                PutCharInFifo(&descrFifoRX, c);
            }
            
                         
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }
        // Traitement controle de flux reception à faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        // ...
            
        if (freeSize <= 6){
            //controle de flux : demande stop émission
            RS232_RTS = 1 ;
        }
    } // end if RX
    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) 
    {
        // Traitement TX à faire ICI
        TXSize = GetReadSize (&descrFifoTX);
        i_cts = RS232_CTS;
        // Envoi des caractères depuis le fifo SW -> buffer HW 
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratères à émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
        TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_1);
        if ((i_cts == 0) && (TXSize > 0) && TxBuffFull == false)
        {
            do {
                GetCharFromFifo (&descrFifoTX, &c);
                PLIB_USART_TransmitterByteSend (USART_ID_1,c);
                i_cts = RS232_CTS;
                TXSize = GetReadSize (&descrFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_1);
                
                } while ((i_cts == 0) && (TXSize > 0) && (TxBuffFull == false));
                
             LED5_W = !LED5_R; // Toggle Led5  
             
		// Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
   
        }
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
 } // end_ISR Usart 1


