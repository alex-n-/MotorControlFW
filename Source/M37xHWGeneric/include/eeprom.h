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

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include "config.h"

#define EEPROM_PAGESIZE 16                                                      /*!< Pagesize of used EEProm */

#define CMD_WRSR        0x01                                                    /*!< Write STATUS register */
#define CMD_WRITE       0x02                                                    /*!< Write data to memory array beginning at selected address */
#define CMD_READ        0x03                                                    /*!< Read data from memory array beginning at selected address */
#define CMD_WRDI        0x04                                                    /*!< Reset the write enable latch (disable write operations) */
#define CMD_RDSR        0x05                                                    /*!< Read STATUS register */
#define CMD_WREN        0x06                                                    /*!< Set the write enable latch (enable write operations) */

#define STATUS_WIP      0x01                                                    /*!< Write-In-Process */
#define STATUS_WEL      0x02                                                    /*!< Write Enable Latch */
#define STATUS_BP0      0x04                                                    /*!< Block Protection */
#define STATUS_BP1      0x08                                                    /*!< Block Protection */

#define BP_NONE         0x00                                                    /*!< Block Protection none */
#define BP_60_7F        0x01                                                    /*!< Block Protection upper quarter */
#define BP_40_7F        0x02                                                    /*!< Block Protection upper half */
#define BP_ALL          0x03                                                    /*!< Block Protection all */

int8_t EEPROM_Read (uint16_t pos, uint8_t* data, uint16_t size);
int8_t EEPROM_Write(uint16_t pos, uint8_t* data, uint16_t size);
#endif
