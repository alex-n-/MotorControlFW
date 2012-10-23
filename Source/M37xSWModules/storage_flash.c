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

#ifdef USE_CONFIG_STORAGE_FLASH
#include "debug.h"
#include BOARD_BOARD_HEADER_FILE
#include TMPM_FC_HEADER_FILE

#include "config_storage.h"
#include "storage_backend.h"

/* FC_BLOCK_0 is the "highest" block for all systems we support */
#define CONFIG_STORE_BLOCK  FC_BLOCK_0
#define NUM_PAGES (FC_GetNumPages(CONFIG_STORE_BLOCK)-1)

#define MAGIC 0xdeadbeef

#define LDEBUG
#ifdef LDEBUG
#define lprintf dprintf
#else
#define lprintf(...)
#endif

static FC_Result _FC_WritePage(uint32_t PageAddr, uint32_t* Data)
{
  FC_Result ret;

  __disable_irq();
  ret = FC_WritePage(PageAddr, Data);
  __enable_irq();

  lprintf("PageAddr 0x%08lux, ret %d\n", PageAddr, ret);

  return ret;
}

static FC_Result _FC_EraseBlock(uint32_t BlockAddr)
{
  FC_Result ret;

  __disable_irq();
  ret = FC_EraseBlock(BlockAddr);
  __enable_irq();

  lprintf("BlockAddr 0x%08lx\n", BlockAddr);

  return ret;
}

/*
static void dump_flash_page(uint8_t *buf) 
{
  uint32_t *ptr = (uint32_t *)buf;
  int i;
  
  dprintf("buf %p\n", buf);
  
  for (i = 0; i < FLASH_PAGE_SIZE/4; i++) {
    printf("%2d: 0x%08x\n", i, *ptr++);
  }
}
*/

static int find_last_valid_config_page(void)
{
  uint32_t base_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  int i;
  
  /* iterate through all the pages and check if config
  data is present by looking for magic at end of the block */
  for (i = 0; i < NUM_PAGES; i++) {
    uint8_t *ptr = (uint8_t *)(base_addr + i * FLASH_PAGE_SIZE);
    uint32_t *magic = (uint32_t *)(ptr + FLASH_PAGE_SIZE - 4);

//    dump_flash_page(ptr);
    
    /* check magic at end of block */
    if (*magic != MAGIC) {
      lprintf("incorrect magic @ offset %d\n", i);
      break;
    }
  }

  /* the last valid config page is the page before the one that had
  the wrong magic and might by -1 if offset 0 is invalid as well */
  i--;
  lprintf("last valid offset is offset %d\n", i);
  return i;
} 

int storage_backend_load(uint8_t *buf, int size, int len)
{
  int offset;
  uint32_t base_addr;
  uint8_t *ptr;

  if (size < CONFIG_STORAGE_PAGE_SIZE) {
    lprintf("size %d < CONFIG_STORAGE_PAGE_SIZE %lud\n", size, CONFIG_STORAGE_PAGE_SIZE);
    return -1;
  }
  
  if(len > CONFIG_STORAGE_MAX_ITEM_SIZE) {
    lprintf("len %d < CONFIG_STORAGE_MAX_ITEM_SIZE %lud\n", len, CONFIG_STORAGE_MAX_ITEM_SIZE);
    return -1;
  }
   
  offset = find_last_valid_config_page();
  if (offset == -1) {
    lprintf("flash does not seem to be initialized\n");
    return -1;
  }

  /* copy config data from the last valid page */
  base_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  ptr = (uint8_t *)(base_addr + offset * FLASH_PAGE_SIZE);
  memcpy(buf, ptr, CONFIG_STORAGE_MAX_ITEM_SIZE);

//  dump_flash_page(ptr);

  lprintf("loaded from offset %d @ %p\n", offset, ptr);
  return 0;
}

static void read_password(uint8_t *password)
{
  /* read password */
  uint32_t base_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  uint8_t *ptr = (uint8_t *)(base_addr + (NUM_PAGES+1) * FLASH_PAGE_SIZE - 12);
  lprintf("pw in flash @ %p\n", ptr);
  memcpy(password, ptr, 12);

#ifdef LDEBUG
  {
  int i;
  lprintf("password is ");
  for (i = 0; i < 12; i++) {
    printf("0x%02x ", password[i]);
  }
  printf("\n");
  }
#endif
}

static int write_password(uint8_t *buf, int size, uint8_t *password)
{
#if 0
  int ret;
  
  if (size < FLASH_PAGE_SIZE) {
    lprintf("size %d < FLASH_PAGE_SIZE %d\n", size, FLASH_PAGE_SIZE);
    return -1;
  }

  memset(buf, 0, size);
  uint8_t *ptr = buf + FLASH_PAGE_SIZE - 12;
  lprintf("pw in buf @ %p\n", ptr);
  memcpy(ptr, password, 12);
    
  /* store password */
  uint32_t dst_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  dst_addr += NUM_PAGES * FLASH_PAGE_SIZE;
  ret = _FC_WritePage(dst_addr, (uint32_t *)buf);
  if (ret) {
    dprintf("_FC_WritePage() failed\n");
    return -1;
  }
  
#ifdef LDEBUG
  {
  int i;
  lprintf("password is ");
  for (i = 0; i < 12; i++) {
    printf("0x%02x ", password[i]);
  }
  printf("\n");
  }
#endif
#endif
  return 0;
}

/* the config area is initialized by clearing it */
static int init_config_area()
{
  uint32_t base_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  FC_Result res;
  
  res = _FC_EraseBlock(base_addr);
  if (res != FC_SUCCESS) {
    dprintf("_FC_EraseBlock() failed\n");
    return -1;
  }
  
  lprintf("erased @ 0x%08lx\n", base_addr);
  return 0;
}

int storage_backend_save(uint8_t *buf, int size, int len)
{
  int offset;
  uint32_t *magic;
  uint32_t dst_addr;
  uint8_t password[12];
  int restore_password = 0;
  int ret;

  if (size < CONFIG_STORAGE_PAGE_SIZE) {
    lprintf("size %d < CONFIG_STORAGE_PAGE_SIZE %lud\n", size, CONFIG_STORAGE_PAGE_SIZE);
    return -1;
  }
  
  if(len > CONFIG_STORAGE_MAX_ITEM_SIZE) {
    lprintf("len %d < CONFIG_STORAGE_MAX_ITEM_SIZE %lud\n", len, CONFIG_STORAGE_MAX_ITEM_SIZE);
    return -1;
  }
   
  offset = find_last_valid_config_page();
  /* special handling: flash is not initialized yet */
  if (offset == -1) {
    lprintf("flash (apparently) not initialized yet\n");

    read_password(&password[0]);
    restore_password = 1;
    
    ret = init_config_area();
    if (ret) {
      dprintf("init_config_area() failed\n");
      return -1;
    }
    offset = 0;
  }
  /* special handling: wrap around */
  else if (offset == NUM_PAGES-1) {
    lprintf("wrap around\n");
    ret = init_config_area();
    if (ret) {
      dprintf("init_config_area() failed\n");
      return -1;
    }
    offset = 0;
  }
  /* otherwise just use the next offset */
  else {
    offset++;
  }

  /* calculate dst address */
  dst_addr = FC_BlockNumToAddr(CONFIG_STORE_BLOCK);
  dst_addr += offset * FLASH_PAGE_SIZE;
  
  /* store magic at end of src data */
  magic = (uint32_t *)(buf + FLASH_PAGE_SIZE - 4);
  *magic = MAGIC;

//  dump_flash_page(buf);
  
  ret = _FC_WritePage(dst_addr, (uint32_t *)buf);
  if (ret) {
    dprintf("_FC_WritePage() failed\n");
    return -1;
  }

  if (restore_password) {
    ret = write_password(buf, size, &password[0]);
    if (ret) {
      dprintf("write_password() failed\n");
      return -1;
    }
  }
    
  lprintf("stored to offset %d @ 0x%08lx\n", offset, dst_addr);
  return 0;
}

static uint8_t password[12]; //  = { 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, };

int storage_backend_clear(uint8_t *buf, int size)
{
  int ret;
  
  read_password(&password[0]);
  
  ret = init_config_area();
  if (ret) {
    dprintf("init_config_area() failed\n");
    return -1;
  }

  ret = write_password(buf, size, &password[0]);
  if (ret) {
    dprintf("write_password() failed\n");
    return -1;
  }

  lprintf("\n");

   return 0;
}

#endif
