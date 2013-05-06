/* THE SOURCE CODE AND ITS
 * RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA CORPORATION MAKES NO OTHER
 * WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR, STATUTORY AND DISCLAIMS ANY
 * AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY, SATISFACTORY QUALITY, NON
 * INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. THE SOURCE CODE AND
 * DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION RESERVES THE RIGHT TO
 * INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER REVISIONS OF IT, AND TO
 * MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR THE PRODUCTS OR
 * TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME. TOSHIBA CORPORATION SHALL NOT BE
 * LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGE OR LIABILITY ARISING
 * FROM YOUR USE OF THE SOURCE CODE OR ANY DOCUMENTATION, INCLUDING BUT NOT
 * LIMITED TO, LOST REVENUES, DATA OR PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL
 * OR CONSEQUENTIAL NATURE, PUNITIVE DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS
 * ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF
 * ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM
 * FOR SUCH DAMAGE IS BASED UPON WARRANTY, CONTRACT, TORT, NEGLIGENCE OR
 * OTHERWISE. (C)Copyright TOSHIBA CORPORATION 2011 All rights reserved
 */

#include <string.h>

#include "config.h"
#ifdef USE_CONFIG_STORAGE

#include "debug.h"
#include "config_storage.h"
#include "storage_backend.h"

#include "motorctrl.h"
#include "crc8.h"

//#define LDEBUG
#ifdef LDEBUG
#define lprintf dprintf
#else
#define lprintf(...)
#endif

extern FWVersion              FirmwareVersion;
extern MotorParameters        MotorParameterValues[MAX_CHANNEL];
extern PIControlSettings      PIControl[MAX_CHANNEL];
extern SystemDependandValues  SystemValues[MAX_CHANNEL];
extern ChannelDependandValues ChannelValues[MAX_CHANNEL];

/* this is correct: the block must be CONFIG_STORAGE_PAGE_SIZE
large, because it's directly used by the storage write function to
read/write one block. the config storage is only allowed to use 
CONFIG_STORAGE_MAX_ITEM_SIZE bytes of this memory, though. */
uint8_t g_config_storage[CONFIG_STORAGE_PAGE_SIZE];

/*! \brief  Read config from storage
  *
  * Read out stored parameter sets from storage and write them to the parameter sets in RAM
  *
  * @param  None
  * @retval Success
*/
int config_storage_load_config(void)
{
  uint8_t   *ptr;
  int       ret;
  uint8_t   crc8;
  FWVersion storedFWVersion;

  if (CONFIG_STORAGE_LEN > CONFIG_STORAGE_MAX_ITEM_SIZE) {
    dprintf("CONFIG_STORAGE_LEN %ud > CONFIG_STORAGE_MAX_ITEM_SIZE %ud\n", CONFIG_STORAGE_LEN, CONFIG_STORAGE_MAX_ITEM_SIZE);
    return -1;
  }
  
  memset(&g_config_storage[0], 0, sizeof(g_config_storage));

  ret = storage_backend_load(&g_config_storage[0], sizeof(g_config_storage), CONFIG_STORAGE_LEN);
  if (ret) {
    dprintf("storage_backend_load() failed, ret %d\n", ret);
    return -1;
  }

  crc8 = crc(&g_config_storage[0], CONFIG_STORAGE_LEN-1);
  if (crc8 != g_config_storage[CONFIG_STORAGE_LEN-1]) {
    dprintf("crc8 mismatch\n");
    return -1;
  }

  ptr = &g_config_storage[0];
  
  memcpy(&storedFWVersion, ptr, sizeof(FWVersion));    
  ptr += sizeof(FWVersion);
  
  if ( (storedFWVersion.fw_version[0] != FirmwareVersion.fw_version[0])
     ||(storedFWVersion.fw_version[1] != FirmwareVersion.fw_version[1]))
  {
    dprintf("Stored FW Version Data missmatch\n");
    return -1;
  }
    
  memcpy(&MotorParameterValues[MOTOR_CHANNEL_FOR_STORAGE], ptr, sizeof(MotorParameters));
  ptr += sizeof(MotorParameters);
  memcpy(&PIControl[MOTOR_CHANNEL_FOR_STORAGE], ptr, sizeof(PIControlSettings));
  ptr += sizeof(PIControlSettings);
  memcpy(&SystemValues[MOTOR_CHANNEL_FOR_STORAGE], ptr, sizeof(SystemDependandValues));
  ptr += sizeof(SystemDependandValues);
#ifdef USE_RW_BOARD_SETTINGS
  memcpy(&ChannelValues[MOTOR_CHANNEL_FOR_STORAGE], ptr, sizeof(ChannelDependandValues));
  ptr += sizeof(ChannelDependandValues);
#endif /* USE_RW_BOARD_SETTINGS */  
  lprintf("LEN %d\n", LEN);
  return 0;
}

/*! \brief  Write config to storage
  *
  * Write the configuration parameters from RAM to storage
  *
  * @param  channel_number: parameters of which channel to write
  * @retval Success
*/
int config_storage_save_config(void)
{
  uint8_t *ptr;
  int ret;
  
  if (CONFIG_STORAGE_LEN > CONFIG_STORAGE_MAX_ITEM_SIZE) {
    dprintf("CONFIG_STORAGE_LEN %ud > CONFIG_STORAGE_MAX_ITEM_SIZE %ud\n", CONFIG_STORAGE_LEN, CONFIG_STORAGE_MAX_ITEM_SIZE);
    return -1;
  }

  memset(&g_config_storage[0], 0, sizeof(g_config_storage));
  ptr = &g_config_storage[0];

  memcpy(ptr, &FirmwareVersion, sizeof(FWVersion));    
  ptr += sizeof(FWVersion);
  memcpy(ptr, &MotorParameterValues[MOTOR_CHANNEL_FOR_STORAGE], sizeof(MotorParameters));
  ptr += sizeof(MotorParameters);
  memcpy(ptr, &PIControl[MOTOR_CHANNEL_FOR_STORAGE], sizeof(PIControlSettings));
  ptr += sizeof(PIControlSettings);
  memcpy(ptr, &SystemValues[MOTOR_CHANNEL_FOR_STORAGE], sizeof(SystemDependandValues));
  ptr += sizeof(SystemDependandValues);
#ifdef USE_RW_BOARD_SETTINGS
  memcpy(ptr, &ChannelValues[MOTOR_CHANNEL_FOR_STORAGE], sizeof(ChannelDependandValues));
  ptr += sizeof(ChannelDependandValues);
#endif /* USE_RW_BOARD_SETTINGS */  
  
  /* store crc8 after all data */
  *ptr++ = crc(&g_config_storage[0], CONFIG_STORAGE_LEN-1);
    
  ret = storage_backend_save(&g_config_storage[0], sizeof(g_config_storage), CONFIG_STORAGE_LEN);
  if (ret) {
    dprintf("STORAGE_EepromSaveConfig() failed, ret %d\n", ret);
    return -1;
  }

  lprintf("CONFIG_STORAGE_LEN %d, crc8 0x%02x\n", CONFIG_STORAGE_LEN, crc8); 
  return 0;
}

/*! \brief  Clear current config in storage
  *
  * Clear the current config in storage
  *
  * @param  None
  * @retval Success
*/
int config_storage_clear_config(void)
{
  int ret;

  ret = storage_backend_clear(&g_config_storage[0], sizeof(g_config_storage));
  if (ret) {
    dprintf("storage_backend_clear() failed\n");
    return -1;
  }

  lprintf("\n");
  return 0;
}

#endif
