/* BSD 3-Clause License
 * Copyright (c) 2017, T. Chyrowicz <tomasz.chyrowicz@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this
 *       list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *       this list of conditions and the following disclaimer in the documentation
 *       and/or other materials provided with the distribution.
 *
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* SE8R01 SPI Commands */
#define CMD_R_REG                     0x00  // 000x xxxx  Read command and status registers (5 bit register map address).
#define CMD_W_REG                     0x20  // 001x xxxx  Write command and status registers (5 bit register map address). Executable in power down or standby modes only.
#define CMD_RW_REG_MASK               0x1F  // 0001 1111  Register address mask.
#define CMD_R_RX_PAYLOAD              0x61  // 0110 0001  Read RX-payload: 1-32 bytes.
#define CMD_W_TX_PAYLOAD              0xA0  // 1010 0000  Write TX-payload: 1 – 32 bytes.
#define CMD_FLUSH_TX                  0xE1  // 1110 0001  Flush TX FIFO.
#define CMD_FLUSH_RX                  0xE2  // 1110 0010  Flush RX FIFO. Should not be executed during transmission of acknowledgement.
#define CMD_REUSE_TX_PAYLOAD          0xE3  // 1110 0011  Used for a PTX operation (Reuse last transmitted payload).
#define CMD_R_RX_PAYLOAD_WIDTH        0x60  // 0110 0000  Read RX payload width for the payload on top of the RX FIFO.
#define CMD_W_ACK_PAYLOAD             0xA8  // 1010 1xxx  Write Payload to be transmitted together with ACK packet on PIPE (xxx).
#define CMD_W_NOACK_PAYLOAD           0xB0  // 1011 0000  Disables AUTOACK on this specific packet.
// W_TX_PAYLOAD_NO_ACK
#define CMD_ACTIVATE                  0x50  // 0101 0000  UNDOCUMENTED: Activate special functions - enter activation mode.
#define CMD_ACTIVATE_SWITCH_BANK      0x53  // 0101 0000  UNDOCUMENTED: Switch to other bank of registers (1 and 0 exists).
#define CMD_ACTIVATE_SWITCH_FEATURES  0x53  // 0101 0000  UNDOCUMENTED: Toggle features.
#define CMD_NOP                       0xFF  // 1111 1111  Define No Operation, might be used to read status register.

/* SE8R01 Bank 0 registers */
#define REG_CONFIG                    0x00  // 'Config' register address
#define REG_EN_AA                     0x01  // 'Enable Auto Acknowledgment' register address
#define REG_EN_RXADDR                 0x02  // 'Enabled RX addresses' register address
#define REG_SETUP_AW                  0x03  // 'Setup address width' register address
#define REG_SETUP_RETR                0x04  // 'Setup Auto. Retrans' register address
#define REG_RF_CH                     0x05  // 'RF channel' register address
#define REG_RF_SETUP                  0x06  // 'RF setup' register address
#define REG_STATUS                    0x07  // 'Status' register address
#define REG_OBSERVE_TX                0x08  // 'Observe TX' register address
#define REG_RPD                       0x09  // 'Received Power Detector' (carrier detector) register address
#define REG_RX_ADDR_P0                0x0A  // 'RX address pipe0' register address
#define REG_RX_ADDR_P1                0x0B  // 'RX address pipe1' register address
#define REG_RX_ADDR_P2                0x0C  // 'RX address pipe2' register address
#define REG_RX_ADDR_P3                0x0D  // 'RX address pipe3' register address
#define REG_RX_ADDR_P4                0x0E  // 'RX address pipe4' register address
#define REG_RX_ADDR_P5                0x0F  // 'RX address pipe5' register address
#define REG_TX_ADDR                   0x10  // 'TX address' register address
#define REG_RX_PW_P0                  0x11  // 'RX payload width, pipe0' register address
#define REG_RX_PW_P1                  0x12  // 'RX payload width, pipe1' register address
#define REG_RX_PW_P2                  0x13  // 'RX payload width, pipe2' register address
#define REG_RX_PW_P3                  0x14  // 'RX payload width, pipe3' register address
#define REG_RX_PW_P4                  0x15  // 'RX payload width, pipe4' register address
#define REG_RX_PW_P5                  0x16  // 'RX payload width, pipe5' register address
#define REG_FIFO_STATUS               0x17  // 'FIFO Status Register' register address
#define REG_DYNPD                     0x1C  // 'Enable dynamic payload length' register address
#define REG_FEATURE                   0x1D  // 'Feature' register address
#define REG_SETUP_VALUE               0x1E  // 'SETUP_VALUE' register address. Configures additional features
#define REG_PRE_GURD                  0x1F  // 'PRE_GURD' register address. Configures frame format

/* SE8R01 Bank 1 registers - not described inside datasheet */
#define REG_LINE                      0x00
#define REG_PLL_CTL0                  0x01
#define REG_PLL_CTL1                  0x02
#define REG_CAL_CTL                   0x03
#define REG_A_CNT_REG                 0x04
#define REG_B_CNT_REG                 0x05
#define REG_RESERVED0                 0x06
#define REG_STATUS                    0x07
#define REG_STATE                     0x08
#define REG_CHAN                      0x09
#define REG_IF_FREQ                   0x0A
#define REG_AFC_COR                   0x0B
#define REG_FDEV                      0x0C
#define REG_DAC_RANGE                 0x0D
#define REG_DAC_IN                    0x0E
#define REG_CTUNING                   0x0F
#define REG_FTUNING                   0x10
#define REG_RX_CTRL                   0x11
#define REG_FAGC_CTRL                 0x12
#define REG_FAGC_CTRL_1               0x13
#define REG_DAC_CAL_LOW               0x17
#define REG_DAC_CAL_HI                0x18
#define REG_RESERVED1                 0x19
#define REG_DOC_DACI                  0x1A
#define REG_DOC_DACQ                  0x1B
#define REG_AGC_CTRL                  0x1C
#define REG_AGC_GAIN                  0x1D
#define REG_RF_IVGEN                  0x1E
#define REG_TEST_PKDET                0x1F

//---------------------------WTF-----------------------
//BITS
#define POWER_BIT                     0x02

//Interrupts
#define IRQ_RX                        0x40
#define IRQ_TX                        0x20
#define IRQ_MAX_RT                    0x10

//FIFO status
#define FIFO_STATUS_TX_REUSE          0x40
#define FIFO_STATUS_TX_FULL           0x20
#define FIFO_STATUS_TX_EMPTY          0x10
#define FIFO_STATUS_RX_FULL           0x02
#define FIFO_STATUS_RX_EMPTY          0x01

//SPEED
#define SPEED_500Kbps                 0x28
#define SPEED_1Mbps                   0x00
#define SPEED_2Mbps                   0x08

//POWER
#define POWER_5dbm                    0x47
#define POWER_0dbm                    0x40
#define POWER_m6dbm                   0x04
#define POWER_m12dbm                  0x02
#define POWER_m18dbm                  0x01

//REG BANKS
#define BANK0                         0x00
#define BANK1                         0x80
//----------------------END-----------------------

/* Bank 1 registers bitwise mnemonics */
#define MASK_RX_DR  6 // NRF_CONFIG
#define MASK_TX_DS  5 // NRF_CONFIG
#define MASK_MAX_RT 4 // NRF_CONFIG
#define EN_CRC      3 // NRF_CONFIG
#define CRCO        2 // NRF_CONFIG
#define PWR_UP      1 // NRF_CONFIG
#define PRIM_RX     0 // NRF_CONFIG
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5 // EN_RXADDR
#define ERX_P4      4 // EN_RXADDR
#define ERX_P3      3 // EN_RXADDR
#define ERX_P2      2 // EN_RXADDR
#define ERX_P1      1 // EN_RXADDR
#define ERX_P0      0 // EN_RXADDR
#define AW          0
#define ARD         4 // SETUP_RETR
#define ARC         0 // SETUP_RETR
#define PLL_LOCK    4
#define RF_DR       3 // RF_SETUP
#define RF_PWR      6 // RF_SETUP
#define RX_DR       6 // NRF_STATUS
#define TX_DS       5 // NRF_STATUS
#define MAX_RT      4 // NRF_STATUS
#define RX_P_NO     1 // NOP
#define TX_FULL     0 // NOP
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4 // FIFO_STATUS
#define RX_FULL     1 // FIFO_STATUS
#define RX_EMPTY    0 // FIFO_STATUS
#define DPL_P5	    5 // DYNPD
#define DPL_P4	    4 // DYNPD
#define DPL_P3	    3 // DYNPD
#define DPL_P2	    2 // DYNPD
#define DPL_P1	    1 // DYNPD
#define DPL_P0	    0 // DYNPD
#define EN_DPL	    2 // FEATURE
#define EN_ACK_PAY  1 // FEATURE
#define EN_DYN_ACK  0 // FEATURE

/* Non-P omissions */
#define LNA_HCURR   0

/* P model bit Mnemonics */
#define RF_DR_LOW   5 // RF_SETUP
#define RF_DR_HIGH  3 // RF_SETUP
#define RF_PWR_LOW  1 // RF_SETUP
#define RF_PWR_HIGH 2 // RF_SETUP
