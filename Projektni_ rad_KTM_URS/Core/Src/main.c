/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#define	SMJER	GPIO_PIN_12	// NEMA 23 SMJER 12
#define	PULS	GPIO_PIN_13	// NEMA 23 PULS 13
#define SMJER1	GPIO_PIN_10	// NEMA 17 SMJER 10
#define PULS1	GPIO_PIN_6	// NEMA 17 PULS 6

#define	RAD_TO_DEG	57.297469 // pretvorba radijana u stupnjeve
#define DEG_TO_RAD	0.0174528 // pretvorba stupnjeva u radijane

#define	BRZINA_DELAY	25 // delay koji regulira brzinu koračnih motora

#define	KRAK	490 // duljina od osi NEMA 23 do osi NEMA 17
#define X0_U_X1	75	// pomak po osi x od ishodišta koord. sustava za unos do ishodišta k. sustava kretanja tereta
#define	Z0_U_Z1	-100	// pomak po osi z od ishodišta koord. sustava za unos do ishodišta k. sustava kretanja tereta
#define R_KOLOT	10	// radijus kolotura na NEMA 17 motoru
#define BLIZINA_DIZALICI	60 // najmanja sigurna vrijednost osi x bez kolizije s dizalicom
#define BLIZINA_MOTORU	50	// najveća sigurna udaljenost tereta od motora NEMA 17

#define	N23_1PULS_DEG	0.09	// iznos za koji se okrene reduktor za 1 puls NEMA 23 motora u stupnjevima
#define N17_1PULS_MM	0.1570796	// iznos pomaka tereta za jedan puls NEMA 17 motora u mm

#define	ADRESA	0x27	// I2C adresa PCF8754 modula
#define	SUCELJE	0x38	//
#define INITIAL	0x03
#define	BIT_4	0x02
#define	EKRAN_0	0x00
#define	CISTI	0x01
#define	KURSOR	0x04
#define	UKLJUCI	0x0F
#define	POCETAK	0x02

#define PRVA_L	0x80	// naredba postavljanje na prvi stupac u prvom retku LCD-a
#define	DRUGA_L	0xC0	// naredba postavljanje na prvi stupac u drugom retku LCD-a
#define	TRECA_L	0x94	// naredba postavljanje na prvi stupac u trećem retku LCD-a
#define CETVR_L	0xD4	// naredba postavljanje na prvi stupac u četvrtom retku LCD-a

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
PUTCHAR_PROTOTYPE	// slanje informacija preko USART2
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
GETCHAR_PROTOTYPE	// prihvaćanje informacija preko USART2
{
	uint8_t ch = 0;
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	HAL_UART_Receive(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t pritisnut_pin = 0;	// varijabla koja prati koji je interrupt pin pritisnut
uint8_t zastava = 0;	// pomocna varijabla za kontroliranje dizalice
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void broj_u_string(short broj, char *string, char tren_ili_cilj, char x_ili_z)
{
	if(!tren_ili_cilj)	// određuje je li se ispisuje tekst za trenutnu ili ciljanu koordinatu
	{
		string[0] = 'T';
		string[1] = 'R';
		string[2] = 'E';
		string[3] = 'N';
		string[4] = 'U';
		string[5] = 'T';
		string[6] = 'N';
		string[7] = 'I';
		string[8] = ' ';
		string[9] = (x_ili_z) ? 'Z' : 'X';	// određuje ispisuje li se za x ili z os
		string[10] = ':';
	}
	else
	{
		string[0] = 'C';
		string[1] = 'I';
		string[2] = 'L';
		string[3] = 'J';
		string[4] = 'A';
		string[5] = 'N';
		string[6] = 'I';
		string[7] = ' ';
		string[8] = (x_ili_z) ? 'Z' : 'X';
		string[9] = ':';
		string[10] = ' ';
	}

	for(int i = 11; i < 14; i++)	// ispisuje razmake u sredini
	{
		string[i] = ' ';
	}

	for(int i = 16; i >= 14; i--)	// pretvara brojeve u ASCII znakove i sprema ih u string
	{
		string[i] = (broj%10) + '0';
		broj /= 10;
	}

	string[17] = ' ';	// dodaje mm na kraju stringa
	string[18] = 'm';
	string[19] = 'm';
}

void lcd_naredba(char podatak)
{
	unsigned char podaci[4], hi, lo; // pomocne varijable za slanje naredbi na LCD

	hi = podatak & 0xF0;	// podjela bajta naredbe na nibbleove
	lo = (podatak << 4) & 0xF0;

	podaci[0] = hi | 0x0C;	// D7, D6, D5, D4, LED = 1, E = 1, R/!W = 0, RS = 0
	podaci[1] = hi | 0x08;	// D7, D6, D5, D4, LED = 1, E = 0, R/!W = 0, RS = 0
	podaci[2] = lo | 0x0C;	// D3, D2, D1, D0, LED = 1, E = 1, R/!W = 0, RS = 0
	podaci[3] = lo | 0x08;	// D3, D2, D1, D0, LED = 1, E = 0, R/!W = 0, RS = 0

	HAL_I2C_Master_Transmit(&hi2c1, ADRESA<<1, podaci, 4, 100); // slanje naredbe na LCD
}

void lcd_podatak(char podatak)
{
	unsigned char podaci[4], hi, lo; // pomocne varijable za slanje podatka na LCD

	hi = podatak & 0xF0;	// podjela bajta podatka na nibbleove
	lo = (podatak << 4) & 0xF0;

	podaci[0] = hi | 0x0D;	// D7, D6, D5, D4, LED = 1, E = 1, R/!W = 0, RS = 1
	podaci[1] = hi | 0x09;	// D7, D6, D5, D4, LED = 1, E = 0, R/!W = 0, RS = 1
	podaci[2] = lo | 0x0D;	// D3, D2, D1, D0, LED = 1, E = 1, R/!W = 0, RS = 1
	podaci[3] = lo | 0x09;	// D3, D2, D1, D0, LED = 1, E = 0, R/!W = 0, RS = 1

	HAL_I2C_Master_Transmit(&hi2c1, ADRESA<<1, podaci, 4, 100);	// slanje podatka na LCD
}

void lcd_string(char podatak[20], uint8_t red)
{
	switch (red){	// određuje u koji red treba doći
	case 1: lcd_naredba(PRVA_L); break;
	case 2: lcd_naredba(DRUGA_L); break;
	case 3: lcd_naredba(TRECA_L); break;
	case 4: lcd_naredba(CETVR_L); break;
	default: lcd_naredba(PRVA_L); break;
	}

	for(int i = 0; i < 20; i++){	// šalje svaki znak stringa na LCD
		lcd_podatak(podatak[i]);
	}

	HAL_Delay(5);
}

void nema23_korak_1_puls(char smjer) // 0.9 stupnja na motoru, 0.09 stupnja na reduktoru
{
	HAL_GPIO_WritePin(GPIOB, SMJER, smjer);	// određuje smjer motora
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, PULS, 0);	// daje impuls upravljaču motora
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, PULS, 1);
	HAL_Delay(1);

	HAL_Delay(BRZINA_DELAY);	// delay zbog ograničavanja brzine
}

void nema23_korak_1_deg(char smjer) // 9.9 stupnja na motoru, 0.99 stupnja na reduktoru
{
	HAL_GPIO_WritePin(GPIOB, SMJER, smjer);	// određuje smjer motora
	HAL_Delay(1);

	for(int i = 0; i < 11; i++)	// daje 11 impulsa upravljaču motora
	{
		HAL_GPIO_WritePin(GPIOB, PULS, 0);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, PULS, 1);
		HAL_Delay(1);

		HAL_Delay(BRZINA_DELAY);	// delay zbog ograničavanja brzine
	}
}


void nema17_korak_1_puls(char smjer) //	0.9 stupnja, 0.16 mm na koloturu
{
	HAL_GPIO_WritePin(GPIOB, SMJER1, smjer); // određuje smjer motora
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, PULS1, 0);	// daje impuls upravljaču motora
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, PULS1, 1);
	HAL_Delay(1);
}

void nema17_korak_1_mm(char smjer) // 5.4 stupnja, 0.94 mm na koloturu
{
	HAL_GPIO_WritePin(GPIOB, SMJER1, smjer);	// određuje smjer motora
	HAL_Delay(1);

	for(int i = 0; i < 6; i++) // daje 6 impulsa motoru
	{
		HAL_GPIO_WritePin(GPIOB, PULS1, 0);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, PULS1, 1);
		HAL_Delay(1);
	}

}


void unos_koordinata(float *x, float *z)	// unos x i z vrijednosti
{
	float provjera;	// pomocne varijable
	short x0, z0;

	do
	{
		printf("X = ");
		scanf("%hd", &x0);	// unos x vrijednosti
		printf("\nZ = ");
		scanf("%hd", &z0);	// unos z vrijednosti
		printf("\n");

		*x = (float)x0;	// pretvorba u float zbog precizonsti tijekom rada
		*z = (float)z0;

		if ((*z) + Z0_U_Z1 > 0)	// provjera valjanosti unesenih koordinata
		{
			provjera = sqrtf(powf(*x + X0_U_X1 - R_KOLOT,2)+powf(*z + Z0_U_Z1,2));
		}
		else
		{
			provjera = KRAK;
		}

	}
	while(provjera > KRAK || (*x) < BLIZINA_DIZALICI);
}

void nema23_v_dolaz_do_unosa(float *x_tren, float x_koord)
{
	float alfa_tren = acosf((*x_tren)/KRAK)*RAD_TO_DEG;	// pretvorba linearnih duljina u kuteve
	float alfa_koord = acosf(x_koord/KRAK)*RAD_TO_DEG;

	if (alfa_tren < alfa_koord)	// dolaz do željenog kuta
	{
		while (alfa_tren < alfa_koord)
		{
		  	nema23_korak_1_puls(1);
		  	alfa_tren += N23_1PULS_DEG;
		}
	}
	else if (alfa_tren > alfa_koord)
	{
		while (alfa_tren > alfa_koord)
		{
		  	nema23_korak_1_puls(0);
		  	alfa_tren -= N23_1PULS_DEG;
		}
	}

	*x_tren = KRAK*cosf(alfa_tren*DEG_TO_RAD);	// pretvorba trenutnog kuta u lin. duljinu
}

void nema17_dolaz_do_unosa_1(float *z_tren, float z_max_tren, float *tren)
{
	*tren = z_max_tren - (*z_tren);	// pretvorba duljine od podloge u duljinu od motora NEMA 17

	if (*tren > 100)
	{
		while (*tren > 100)	// dolaz do sigurne pozicije da se ne dogodi kolizija s podlogom
		{
			nema17_korak_1_puls(0);
		  	*tren -= N17_1PULS_MM;
		}
	}

	*z_tren = z_max_tren - (*tren);	// pretvorba natrag u duljinu od podloge
}

void nema17_dolaz_do_unosa_2(float *z_tren, float z_koord, float z_max, float *tren)
{
	float koord_2 = z_max - z_koord;	// pretvorba duljine od podloge u duljinu od motora NEMA 17

	if (*tren < koord_2)	// dolaz do željene pozicije
	{
		while (*tren < koord_2)
		{
			nema17_korak_1_puls(1);
		  	*tren += N17_1PULS_MM;

		}
	}
	else if (*tren > koord_2)
	{
		while (*tren > koord_2)
		{
			nema17_korak_1_puls(0);
		  	*tren -= N17_1PULS_MM;

		  	if(*tren < BLIZINA_MOTORU)	// zaštita od kolizije tereta s motorom NEMA 17
		  	{
		  		break;
		  	}
		}
	}

	*z_tren = z_max - (*tren);	// pretvorba natrag u duljinu od podloge
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setvbuf(stdin, NULL, _IONBF, 0);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /*######################################################################################
   *##########VARIJABLE - POCETAK#########################################################
   *###################################################################################### */

  char str[20];	// pomoćna varijabla za ispis stringa
  // sve varijable korištene za pozicioniranje tereta
  float r_tren, x0_koord, z0_koord, x1_koord, z1_koord, z_max_koord, z_max_tren, x0_tren, z0_tren, x1_tren, z1_tren;

  /*######################################################################################
   *##########VARIJABLE - KRAJ############################################################
   *###################################################################################### */

  /*######################################################################################
   *##########INICIJALIZACIJA LCD-a#######################################################
    ###################################################################################### */

  HAL_Delay(15);
  lcd_naredba(INITIAL);	// prve tri naredbe za inicijalizaciju

  HAL_Delay(5);
  lcd_naredba(INITIAL);

  HAL_Delay(1);
  lcd_naredba(INITIAL);

  HAL_Delay(1);
  lcd_naredba(BIT_4);	// konfiguracija LCD-a na 4-bitni mod

  lcd_naredba(SUCELJE);	// DL = 1, N = 1, F = 0, duljina podataka 8, 2 prikazne linije,  font 5x7

  lcd_naredba(CISTI);
  lcd_naredba(EKRAN_0);

  lcd_naredba(EKRAN_0);
  lcd_naredba(CISTI);

  lcd_naredba(KURSOR);	// ID = 0, S = 0, ne inkrementira pokazivač ni prikaz

  lcd_naredba(UKLJUCI);	// D = 1, C = 1, B = 1, uključi prikaz, pokazivač i blinkanje kursora

  HAL_Delay(5);

  lcd_naredba(POCETAK); // stavlja pokazivač na pocetak

  HAL_Delay(2);

  lcd_naredba(CISTI);	// pocisti LCD

  HAL_Delay(2);

/*######################################################################################
 *##########INICIJALIZACIJA ZAVRŠENA####################################################
  ###################################################################################### */

  unos_koordinata(&x0_tren, &z0_tren);	// unos početne pozicije tereta

  broj_u_string((short)x0_tren,str,0,0);	// slanje početne trenutne pozicije tereta
  lcd_string(str,1);
  broj_u_string((short)z0_tren,str,0,1);
  lcd_string(str,2);

  broj_u_string(0,str,1,0);	// slanje početne željenje pozicije tereta (0,0)
  lcd_string(str,3);
  broj_u_string(0,str,1,1);
  lcd_string(str,4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  if (zastava == 1)	// ulaz u automatski mod dizalice
	  	  {
	  		  unos_koordinata(&x0_koord, &z0_koord);	// unos željene pozicije tereta

	  		  broj_u_string((short)x0_koord,str,1,0);	// ispis željene pozicije tereta
	  		  lcd_string(str,3);
	  		  broj_u_string((short)z0_koord,str,1,1);
	  		  lcd_string(str,4);

	  		  x1_koord = x0_koord + X0_U_X1 - R_KOLOT;	// mijenjanje koordinatnog sustava
	  		  x1_tren = x0_tren + X0_U_X1 - R_KOLOT;
	  		  z1_koord = z0_koord + Z0_U_Z1;
	  		  z1_tren = z0_tren + Z0_U_Z1;

	  		  z_max_koord = KRAK*sinf(acosf(x1_koord/KRAK));	// izracun pozicije NEMA 17 motora
	  		  z_max_tren = KRAK*sinf(acosf(x1_tren/KRAK));

	  		  nema17_dolaz_do_unosa_1(&z1_tren, z_max_tren, &r_tren);	// prvotno dizanje tereta

	  		  nema23_v_dolaz_do_unosa(&x1_tren, x1_koord);	// dolaz do željene x pozicije

	  		  nema17_dolaz_do_unosa_2(&z1_tren, z1_koord, z_max_koord, &r_tren);	// dolaz do željene z pozicije

	  		  x0_tren = x1_tren - X0_U_X1 + R_KOLOT;	// mijenjanje koordinatnog sustava
	  		  z0_tren = z1_tren - Z0_U_Z1;

	  		  broj_u_string((short)x0_tren,str,0,0);	// ispis trenutnih vrijednosti nakon dolaska na željene koordinate
	  		  lcd_string(str,1);
	  		  broj_u_string((short)z0_tren,str,0,1);
	  		  lcd_string(str,2);

	  		  zastava = 0;
	  	  }
	  	  else if (zastava == 2)	// pomak NEMA 23 motora za 1 stupanj prema gore
	  	  {
	  		  nema23_korak_1_deg(1);

	  		  x1_tren = x0_tren + X0_U_X1 - R_KOLOT;	// izračun promjene x i z koordinati
	  		  z1_tren = z0_tren + Z0_U_Z1;
	  		  x1_tren = KRAK*cosf(acosf(x1_tren/KRAK) + 1*DEG_TO_RAD);
	  		  z1_tren = KRAK*sinf(acosf(x1_tren/KRAK) + 1*DEG_TO_RAD);
	  		  x0_tren = x1_tren - X0_U_X1 + R_KOLOT;
	  		  z0_tren = z1_tren - Z0_U_Z1;

	  		  broj_u_string((short)x0_tren,str,0,0);	// ispis promijenjenih x i z koordinate
	  		  lcd_string(str,1);
	  		  broj_u_string((short)z0_tren,str,0,1);
	  		  lcd_string(str,2);

	  		  zastava = 0;
	  	  }
	  	  else if (zastava == 3)	// pomak NEMA 23 motora za 1 stupanj prema dolje
	  	  {
	  		  nema23_korak_1_deg(0);

	  		  x1_tren = x0_tren + X0_U_X1 - R_KOLOT;	// izračun promjene x i z koordinati
	  		  z1_tren = z0_tren + Z0_U_Z1;
	  		  x1_tren = KRAK*cosf(acosf(x1_tren/KRAK) - 1*DEG_TO_RAD);
	  		  z1_tren = KRAK*sinf(acosf(x1_tren/KRAK) - 1*DEG_TO_RAD);
	  		  x0_tren = x1_tren - X0_U_X1 + R_KOLOT;
	  		  z0_tren = z1_tren - Z0_U_Z1;

	  		  broj_u_string((short)x0_tren,str,0,0);	// ispis promijenjenih x i z koordinati
	  		  lcd_string(str,1);
	  		  broj_u_string((short)z0_tren,str,0,1);
	  		  lcd_string(str,2);

	  		  zastava = 0;
	  	  }
	  	  else if (zastava == 4)	// pomak tereta za 1 milimetar prema gore
	  	  {
	  		  nema17_korak_1_mm(0);

	  		  z0_tren++;	// promjena z koordinate

	  		  broj_u_string((short)z0_tren,str,0,1);	// ispis promijenjene z koordinate
	  		  lcd_string(str,2);

	  		  zastava = 0;
	  	  }
	  	  else if (zastava == 5)	// pomak tereta za 1 milimetar prema dolje
	  	  {
	  		  nema17_korak_1_mm(1);

	  		  z0_tren--;	// promjena z koordinate

	  		  broj_u_string((short)z0_tren,str,0,1);	// ispis promijenjene z koordinate
	  		  lcd_string(str,2);

	  		  zastava = 0;
	  	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	pritisnut_pin = GPIO_Pin;	// saznaje koji je pin pokrenuo interrupt

	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	// micanje interrupta s početka countera
	HAL_TIM_Base_Start_IT(&htim6);	// pokreće counter
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		if (HAL_GPIO_ReadPin(GPIOC, pritisnut_pin) == GPIO_PIN_RESET)
		{	// daje vrijednost zastavici ovisno o tome koji je pin pritisnut
			switch (pritisnut_pin)
			{
				case (GPIO_PIN_5): zastava = 1; break;
				case (GPIO_PIN_6): zastava = 2; break;
				case (GPIO_PIN_7): zastava = 3; break;
				case (GPIO_PIN_8): zastava = 4; break;
				case (GPIO_PIN_9): zastava = 5; break;
			}
		}
		HAL_TIM_Base_Stop_IT(&htim6);	// zaustavlja counter
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
