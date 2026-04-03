/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    eth.c
  * @brief   This file provides code for the configuration
  *          of the ETH instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "eth.h"
#include "string.h"

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_TxPacketConfig TxConfig;

/* USER CODE BEGIN 0 */
// RX/TX buffers for EtherCAT
// Aligned to 4 bytes for DMA
__attribute__((aligned(4))) uint8_t RxBuff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE];
__attribute__((aligned(4))) uint8_t TxBuff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE];

// LAN8720A PHY Address (typically 0 or 1)
// Some modules use 0x00, others use 0x01
// Auto-detect will try both
uint32_t LAN8720A_PHY_ADDRESS = 0x00;

// PHY Special Control/Status Register for LAN8720A
#define PHY_SCSR                0x1F    /* Special Control/Status Register */

#define PHY_SCSR_SPEED_MASK     0x001C
#define PHY_SCSR_SPEED_10_HALF  0x0004
#define PHY_SCSR_SPEED_10_FULL  0x0014
#define PHY_SCSR_SPEED_100_HALF 0x0008
#define PHY_SCSR_SPEED_100_FULL 0x0018

/* USER CODE END 0 */

// External function declaration for UART output
extern void UART_SendLine(const char *str);
extern void UART_SendString(const char *str);

// Buffer for sprintf
static char eth_buf[128];

ETH_HandleTypeDef heth;

// Read PHY register
static uint32_t PHY_ReadRegister(uint32_t RegAddr)
{
    uint32_t readval = 0;
    if (LAN8720A_PHY_ADDRESS != 0xFF) {
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, RegAddr, &readval);
    }
    return readval;
}

// Write PHY register
static void PHY_WriteRegister(uint32_t RegAddr, uint32_t Value)
{
    if (LAN8720A_PHY_ADDRESS != 0xFF) {
        HAL_ETH_WritePHYRegister(&heth, LAN8720A_PHY_ADDRESS, RegAddr, Value);
    }
}

// Auto-detect PHY address
static int PHY_DetectAddress(void)
{
    uint32_t regvalue;
    
    // Try address 0
    HAL_ETH_ReadPHYRegister(&heth, 0, 2, &regvalue);
    if (regvalue != 0 && regvalue != 0xFFFF) {
        LAN8720A_PHY_ADDRESS = 0;
        return 0;
    }
    
    // Try address 1
    HAL_ETH_ReadPHYRegister(&heth, 1, 2, &regvalue);
    if (regvalue != 0 && regvalue != 0xFFFF) {
        LAN8720A_PHY_ADDRESS = 1;
        return 1;
    }
    
    // Default to 0 if not found
    LAN8720A_PHY_ADDRESS = 0;
    return 0;
}

// Initialize LAN8720A PHY
static int LAN8720A_Init(void)
{
    uint32_t regvalue = 0;
    uint32_t tickstart = HAL_GetTick();

    // Auto-detect PHY address
    int phy_addr = PHY_DetectAddress();
    (void)phy_addr;  // Suppress unused warning

    // Reset PHY
    PHY_WriteRegister(PHY_BCR, PHY_RESET);

    // Wait for reset complete
    tickstart = HAL_GetTick();
    do
    {
        regvalue = PHY_ReadRegister(PHY_BCR);
        if ((HAL_GetTick() - tickstart) > 1000)
        {
            return -1;  // Timeout
        }
    } while (regvalue & PHY_RESET);

    // Enable auto-negotiation and wait for link
    UART_SendLine("Enabling auto-negotiation...");
    PHY_WriteRegister(PHY_BCR, PHY_AUTONEGOTIATION);

    // Wait for auto-negotiation complete and link up
    tickstart = HAL_GetTick();
    do
    {
        regvalue = PHY_ReadRegister(PHY_BSR);
        if ((HAL_GetTick() - tickstart) > 5000)
        {
            UART_SendLine("Auto-negotiation timeout, forcing 100Mbps Full Duplex...");
            PHY_WriteRegister(PHY_BCR, 0x2100);  // Force 100Mbps Full Duplex
            HAL_Delay(100);
            break;
        }
    } while (!(regvalue & PHY_LINKED_STATUS));

    // Check final link status
    uint32_t bcr = PHY_ReadRegister(PHY_BCR);
    uint32_t bsr = PHY_ReadRegister(PHY_BSR);
    uint32_t scsr = PHY_ReadRegister(PHY_SCSR);
    sprintf(eth_buf, "PHY BCR=0x%04X BSR=0x%04X SCSR=0x%04X", bcr, bsr, scsr);
    UART_SendLine(eth_buf);
    
    // Parse SCSR for speed and duplex
    uint32_t speed = scsr & PHY_SCSR_SPEED_MASK;
    switch(speed) {
        case PHY_SCSR_SPEED_100_FULL:
            UART_SendLine("  -> Speed: 100Mbps Full Duplex");
            break;
        case PHY_SCSR_SPEED_100_HALF:
            UART_SendLine("  -> Speed: 100Mbps Half Duplex");
            break;
        case PHY_SCSR_SPEED_10_FULL:
            UART_SendLine("  -> Speed: 10Mbps Full Duplex");
            break;
        case PHY_SCSR_SPEED_10_HALF:
            UART_SendLine("  -> Speed: 10Mbps Half Duplex");
            break;
        default:
            sprintf(eth_buf, "  -> Speed: Unknown (SCSR=0x%04X)", speed);
            UART_SendLine(eth_buf);
            break;
    }
    
    if (bsr & PHY_LINKED_STATUS) {
        UART_SendLine("Link is UP");
    } else {
        UART_SendLine("WARNING: Link is DOWN");
    }

    return 0;
}

// Get PHY link status
int ETH_GetLinkStatus(void)
{
    uint32_t bsr = PHY_ReadRegister(PHY_BSR);
    return (bsr & PHY_LINKED_STATUS) ? 1 : 0;
}

// RX buffer index for callback
static volatile uint32_t rx_buffer_index = 0;

// HAL ETH RX Allocate Callback - called by HAL to get RX buffer
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    // Assign next RX buffer
    *buff = RxBuff[rx_buffer_index];
    
    // Increment index for next allocation
    rx_buffer_index = (rx_buffer_index + 1) % ETH_RX_DESC_CNT;
}

/* ETH init function */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  // Register RX allocate callback for buffer management
  HAL_ETH_RegisterRxAllocateCallback(&heth, HAL_ETH_RxAllocateCallback);

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  // Initialize LAN8720A PHY
  LAN8720A_Init();

  // Configure MAC to receive all frames (Promiscuous mode for EtherCAT)
  // Set RA (Receive All) bit in MACFFR to receive all frames including broadcast
  heth.Instance->MACFFR = ETH_MACFFR_RA;

  // Start Ethernet MAC and DMA using HAL library function
  // This properly initializes descriptors and starts reception
  // Use interrupt mode for better real-time performance
  if (HAL_ETH_Start_IT(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Enable ETH interrupt in NVIC
  HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);

  /* USER CODE END ETH_Init 2 */

}

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* ETH clock enable */
    __HAL_RCC_ETH_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ETH_MspInit 1 */

  /* USER CODE END ETH_MspInit 1 */
  }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{

  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
