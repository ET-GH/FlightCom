/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_descriptors.c
  * @author  MCD Application Team
  * @brief   USBX Device descriptor header file
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
#include "ux_device_descriptors.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

uint8_t UserClassInstance[USBD_MAX_CLASS_INTERFACES] = {
  CLASS_TYPE_HID,
};

/* USER CODE BEGIN PV0 */

/* USER CODE END PV0 */

/* String Device Framework :
 Byte 0 and 1 : Word containing the language ID : 0x0904 for US
 Byte 2       : Byte containing the index of the descriptor
 Byte 3       : Byte containing the length of the descriptor string
*/
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN UCHAR USBD_string_framework[USBD_STRING_FRAMEWORK_MAX_LENGTH]
__ALIGN_END = {0};

/* Multiple languages are supported on the device, to add
   a language besides English, the Unicode language code must
   be appended to the language_id_framework array and the length
   adjusted accordingly. */

#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN UCHAR USBD_language_id_framework[LANGUAGE_ID_MAX_LENGTH]
__ALIGN_END = {0};

/* USER CODE BEGIN PV1 */

/* USER CODE END PV1 */

/* Private function prototypes -----------------------------------------------*/
static void USBD_Desc_GetString(uint8_t *desc, uint8_t *Buffer, uint16_t *len);
static uint8_t USBD_Desc_GetLen(uint8_t *buf);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_Get_Device_Framework_Speed
  *         Return the device speed descriptor
  * @param  Speed : HIGH or FULL SPEED flag
  * @param  length : length of HIGH or FULL SPEED array
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_Get_Device_Framework_Speed(uint8_t Speed, ULONG *Length)
{
  uint8_t *pFrameWork = NULL;
  /* USER CODE BEGIN Device_Framework0 */

  /* USER CODE END Device_Framework0 */

  /* USER CODE BEGIN Device_Framework1 */

  /* USER CODE END Device_Framework1 */
  return pFrameWork;
}

/**
  * @brief  USBD_Get_String_Framework
  *         Return the language_id_framework
  * @param  Length : Length of String_Framework
  * @retval Pointer to language_id_framework buffer
  */
uint8_t *USBD_Get_String_Framework(ULONG *Length)
{
  uint16_t len = 0U;
  uint8_t count = 0U;

  /* USER CODE BEGIN String_Framework0 */

  /* USER CODE END String_Framework0 */

  /* Set the Manufacturer language Id and index in USBD_string_framework */
  USBD_string_framework[count++] = USBD_LANGID_STRING & 0xFF;
  USBD_string_framework[count++] = USBD_LANGID_STRING >> 8;
  USBD_string_framework[count++] = USBD_IDX_MFC_STR;

  /* Set the Manufacturer string in string_framework */
  USBD_Desc_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_string_framework + count, &len);

  /* Set the Product language Id and index in USBD_string_framework */
  count += len + 1;
  USBD_string_framework[count++] = USBD_LANGID_STRING & 0xFF;
  USBD_string_framework[count++] = USBD_LANGID_STRING >> 8;
  USBD_string_framework[count++] = USBD_IDX_PRODUCT_STR;

  /* Set the Product string in USBD_string_framework */
  USBD_Desc_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_string_framework + count, &len);

  /* Set Serial language Id and index in string_framework */
  count += len + 1;
  USBD_string_framework[count++] = USBD_LANGID_STRING & 0xFF;
  USBD_string_framework[count++] = USBD_LANGID_STRING >> 8;
  USBD_string_framework[count++] = USBD_IDX_SERIAL_STR;

  /* Set the Serial number in USBD_string_framework */
  USBD_Desc_GetString((uint8_t *)USBD_SERIAL_NUMBER, USBD_string_framework + count, &len);

  /* USER CODE BEGIN String_Framework1 */

  /* USER CODE END String_Framework1 */

  /* Get the length of USBD_string_framework */
  *Length = strlen((const char *)USBD_string_framework);

  return USBD_string_framework;
}

/**
  * @brief  USBD_Get_Language_Id_Framework
  *         Return the language_id_framework
  * @param  Length : Length of Language_Id_Framework
  * @retval Pointer to language_id_framework buffer
  */
uint8_t *USBD_Get_Language_Id_Framework(ULONG *Length)
{
  uint8_t count = 0U;

  /* Set the language Id in USBD_language_id_framework */
  USBD_language_id_framework[count++] = USBD_LANGID_STRING & 0xFF;
  USBD_language_id_framework[count++] = USBD_LANGID_STRING >> 8;

  /* Get the length of USBD_language_id_framework */
  *Length = strlen((const char *)USBD_language_id_framework);

  return USBD_language_id_framework;
}

/**
  * @brief  USBD_Get_Interface_Number
  *         Return interface number
  * @param  class_type : Device class type
  * @param  interface_type : Device interface type
  * @retval interface number
  */
uint16_t USBD_Get_Interface_Number(uint8_t class_type, uint8_t interface_type)
{
  uint8_t itf_num = 0U;

  /* USER CODE BEGIN USBD_Get_Interface_Number0 */

  /* USER CODE END USBD_Get_Interface_Number0 */

  /* USER CODE BEGIN USBD_Get_Interface_Number1 */

  /* USER CODE END USBD_Get_Interface_Number1 */

  return itf_num;
}

/**
  * @brief  USBD_Get_Configuration_Number
  *         Return configuration number
  * @param  class_type : Device class type
  * @param  interface_type : Device interface type
  * @retval configuration number
  */
uint16_t USBD_Get_Configuration_Number(uint8_t class_type, uint8_t interface_type)
{
  uint8_t cfg_num = 1U;

  /* USER CODE BEGIN USBD_Get_CONFIGURATION_Number0 */

  /* USER CODE END USBD_Get_CONFIGURATION_Number0 */

  /* USER CODE BEGIN USBD_Get_CONFIGURATION_Number1 */

  /* USER CODE END USBD_Get_CONFIGURATION_Number1 */

  return cfg_num;
}

#if USBD_HID_CLASS_ACTIVATED == 1U
/**
  * @brief  USBD_HID_ReportDesc
  *         Return the device HID Report Descriptor
  * @param  hid_type : HID Device type
  * @retval Pointer to HID Report Descriptor buffer
  */
uint8_t *USBD_HID_ReportDesc(uint8_t hid_type)
{
  uint8_t *pHidReportDesc = NULL;

  /* USER CODE BEGIN HidReportDesc0 */

  /* USER CODE END HidReportDesc0 */

  switch(hid_type)
  {
    default:
      break;
  }

  /* USER CODE BEGIN HidReportDesc1 */

  /* USER CODE END HidReportDesc1 */

  return pHidReportDesc;
}

/**
  * @brief  USBD_HID_ReportDesc_length
  *         Return the device HID Report Descriptor
  * @param  hid_type : HID Device type
  * @retval Size of HID Report Descriptor buffer
  */
uint16_t USBD_HID_ReportDesc_length(uint8_t hid_type)
{
  uint16_t ReportDesc_Size = 0;

  /* USER CODE BEGIN ReportDesc_Size0 */

  /* USER CODE END ReportDesc_Size0 */

  switch(hid_type)
  {
    default:
      break;
  }

  /* USER CODE BEGIN ReportDesc_Size1 */

  /* USER CODE END ReportDesc_Size1 */

  return ReportDesc_Size;
}
#endif /* USBD_HID_CLASS_ACTIVATED == 1U */

/**
  * @brief  USBD_Desc_GetString
  *         Convert ASCII string into Unicode one
  * @param  desc : descriptor buffer
  * @param  Unicode : Formatted string buffer (Unicode)
  * @param  len : descriptor length
  * @retval None
  */
static void USBD_Desc_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;
  uint8_t *pdesc;

  if (desc == NULL)
  {
    return;
  }

  pdesc = desc;
  *len = (uint16_t)USBD_Desc_GetLen(pdesc);

  unicode[idx++] = *(uint8_t *)len;

  while (*pdesc != (uint8_t)'\0')
  {
    unicode[idx++] = *pdesc;
    pdesc++;
  }
}

/**
  * @brief  USBD_Desc_GetLen
  *         return the string length
  * @param  buf : pointer to the ASCII string buffer
  * @retval string length
  */
static uint8_t USBD_Desc_GetLen(uint8_t *buf)
{
  uint8_t  len = 0U;
  uint8_t *pbuff = buf;

  while (*pbuff != (uint8_t)'\0')
  {
    len++;
    pbuff++;
  }

  return len;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
