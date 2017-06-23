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

 /** @file
  *
  * @defgroup drv_abstract Driver abstract class declaration
  * @{
  * @ingroup drv_common
  *
  * @brief Abstract class declaration for Arduino drivers
  *
  * @details Defines mandatory methods in every Arduino driver. All parsers and block generators will
  *          use this class structure as a starting point.
  */

 #ifndef SE8R01_H__
 #define SE8R01_H__

 #include <Arduino.h>

 /*
  * If driver is using some peripherals, please make a proper locks inside driver_config.h header file
  */
//#include "SE8R01_config.h"

class SE8R01
{
public:
    /**
     * @name Primary public interface
     *
     * @details These are the main and most common methods you need to operate the chip.
     */
    /**@{*/

    /**
     * @brief Arduino Constructor
     *
     * @details Creates a new instance of this driver. Please specify a unique set of pins to which
     *          the device is connected to.
     *
     * @param[in] cepin The pin attached to Chip Enable on the RF module
     * @param[in] cspin The pin attached to Chip Select
     */
    SE8R01(uint16_t cepin, uint16_t cspin);

    /**
     * @brief Begin operation of the chip
     *
     * @detail Call this in setup(), before calling any other methods. Configure IO pins and SPI bus.
     *
     * @retval true   Device is successfully configured and ready.
     * @retval false  Configuration or initialization procedure failed.
     */
    bool begin(void);

    /**
     * @brief Checks if the chip is connected to the SPI bus
     *
     * @retval true   Device is connected and ready.
     * @retval false  Device is disconnected.
     */
    bool isChipConnected(void);

    /**@}*/
    /**
     * @name Basic radio operations.
     *
     * @details Methods for listening, sending and receiving data.
     */
    /**@{*/

    /**
     * @brief Start listening on the pipes opened for reading.
     *
     * @detail Please make sure:
     *             - To call openReadingPipe() first.
     *             - Do not call write() while in this mode, without first calling stopListening().
     *             - Call available() to check for incoming traffic, and read() to get it.
     */
    void startListening(void);

    /**
     * @brief Stop listening for incoming messages, and switch to transmit mode.
     *
     * @detail Do this before calling write().
     */
    void stopListening(void);

    /**
     * @brief Open a pipe for writing via address passed as a byte array.
     *
     * @details Only one writing pipe can be open at once, but you can change the address
     *          you'll write to. Call stopListening() first.
     *          Addresses are assigned via a byte array, default is 5 byte address length
     *          @see setAddressWidth
     *
     * @param[in] address The address of the pipe to open. Coordinate these pipe
     *                    addresses amongst nodes on the network.
     */
    void openWritingPipe(const uint8_t *address);

    /**
     * @brief Open a pipe for reading
     *
     * @detail Up to 6 pipes can be open for reading at once.  Open all the required
     *         reading pipes, and then call startListening().
     *         @see openWritingPipe
     *         @see setAddressWidth
     *
     * @note Pipes 0 and 1 will store a full 5-byte address. Pipes 2-5 will technically
     *       only store a single byte, borrowing up to 4 additional bytes from pipe #1 per the
     *       assigned address width.
     *
     * @warning Pipes 1-5 should share the same address, except the first byte.
     *          Only the first byte in the array should be unique.
     *
     * @warning Pipe 0 is also used by the writing pipe.  So if you open
     *          pipe 0 for reading, and then startListening(), it will overwrite the
     *          writing pipe.  Ergo, do an openWritingPipe() again before write().
     *
     * @param[in] number  Which pipe# to open, 0-5.
     * @param[in] address The 24, 32 or 40 bit address of the pipe to open.
     */
    void openReadingPipe(uint8_t number, const uint8_t *address);

    /**
     * @brief Test whether there are bytes available to be read in the FIFO buffers.
     *
     * @param[out] pipe_num Which pipe has the payload available
     *
     * @return The amount of bytes available
     */
    uint16_t available(uint8_t* pipe_num);

    /**
     * @brief Read available data
     *
     * @detail The size of data read is the fixed payload size, see getPayloadSize().
     *
     * @param[out] p_buffer   Pointer to a buffer where the data should be written
     * @param[in]  length     Maximum number of bytes to read into the buffer
     *
     * @return The amount of bytes written inside the buffer.
     */
    uint16_t read(void * p_buffer, uint16_t length);

    /**
     * @brief Writes or sends data
     *
     * @detail  Be sure to call openWritingPipe() first to set the destination
     *          of where to write to.
     *          This blocks until the message is successfully acknowledged by
     *          the receiver or the timeout/retransmit maximum is reached.  In
     *          the current configuration, the max delay here is 60-70ms.
     *
     *          The maximum size of data written is the fixed payload size, see
     *          getPayloadSize().  However, you can write less, and the remainder
     *          will just be filled with zeros.
     *
     *          TX/RX/RT interrupt flags will be cleared every time write is called
     *
     * @param[in] p_buffer Pointer to the data to be written/sent
     * @param[in] length   Number of bytes to be written/sent
     *
     * @return True if the payload was delivered successfully false if not
     */
    bool write(void * p_buffer, uint16_t length);

    /**
     * @brief Print a giant block of debugging information to stdout
     *
     * @details Prints out device configuration and status registers.
     *
     * @warning In order to use this method, the printf function and stdout have to be defined.
     *          Usually you have to include printf.h file.
     */
    void printDetails(void);

    /**
     * @brief Enter low-power mode
     *
     * @details To return to normal power mode, call powerUp(). Whenever the device provides low
     *          power mode, this method should be used to enter it.
     *          After calling startListening(), a basic radio will consume about 13.5mA at max PA
     *          level. During active transmission, the radio will consume about 11.5mA, but this will
     *          be reduced to 26uA (.026mA) between sending.
     *          In full powerDown mode, the radio will consume approximately 900nA (.0009mA)
     */
    void powerDown(void);

    /**
     * @brief Leave low-power mode
     *
     * @details Required for normal radio operation after calling powerDown(). This will take up to
     *          5ms for maximum compatibility.
     */
    void powerUp(void);


    /**@}*/
    /**
     * @name Advanced Operation
     *
     *  Methods you can use to drive the chip in more advanced ways
     */
    /**@{*/

    /**
     * @brief Check if the radio needs to be read. Can be used to prevent data loss.
     *
     * @return True if all three 32-byte radio buffers are full
     */
    bool rxFifoFull(void);

    /**
     * @brief Write for single NOACK writes.
     *
     * @detail Optionally disables acknowledgements/autoretries for a single write. Can be used with
     *         enableAckPayload() to request a response.
     *
     * @note enableDynamicAck() must be called to enable this feature
     *
     * @param[in] p_buffer  Pointer to the data to be sent
     * @param[in] length    Number of bytes to be sent
     * @param[in] multicast Request ACK (0), NOACK (1)
     */
    bool write(const void* p_buffer, uint8_t length, const bool multicast);

    /**
     * @brief Non-blocking FIFO write.
     *
     * @detail This will not block until the 3 FIFO buffers are filled with data.
     *         Once the FIFOs are full, writeFast will simply wait for success or
     *         timeout, and return 1 or 0 respectively. From a user perspective, just
     *         keep trying to send the same data. The library will keep auto retrying
     *         the current payload using the built in functionality.
     *
     * @warning It is important to never keep the nRF24L01 in TX mode and FIFO full for more than
     *          4ms at a time. If the auto retransmit is enabled, the nRF24L01 is never in TX mode
     *          long enough to disobey this rule. Allow the FIFO to clear by issuing txStandBy() or
     *          ensure appropriate time between transmissions.
     *
     * @param[in] p_buffer Pointer to the data to be sent
     * @param[in] length   Number of bytes to be sent
     *
     * @return True if the payload was delivered successfully false if not
     */
    bool writeFast(const void* p_buffer, uint8_t length);

    /**
     * @brief WriteFast for single NOACK writes. Disables acknowledgements/autoretries for a single write.
     *
     * @note enableDynamicAck() must be called to enable this feature
     *
     * @param[in] p_buffer  Pointer to the data to be sent
     * @param[in] length    Number of bytes to be sent
     * @param[in] multicast Request ACK (0) or NOACK (1)
     */
    bool writeFast(const void* p_buffer, uint8_t length, const bool multicast);

    /**
     * @brief Blocking write with an arbitrary long timeout period.
     *
     * @detail This function extends the auto-retry mechanism to any specified duration.
     *         It will not block until the 3 FIFO buffers are filled with data.
     *         If so the library will auto retry until a new payload is written
     *         or the user specified timeout period is reached.
     *
     * @warning It is important to never keep the nRF24L01 in TX mode and FIFO full for more than 4ms at a time. If the auto
     *          retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
     *          to clear by issuing txStandBy() or ensure appropriate time between transmissions.
     *
     * @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
     *
     * @param[in] p_buffer Pointer to the data to be sent
     * @param[in] length   Number of bytes to be sent
     * @param[in] timeout  User defined timeout in milliseconds.
     *
     * @return True if the payload was loaded into the buffer successfully false if not
     */
    bool writeBlocking(const void* p_buffer, uint8_t length, uint32_t timeout);

    /**
     * @brief PUt device into STANDBY-I mode.
     *
     * @detail This function should be called as soon as transmission is finished to
     *         drop the radio back to STANDBY-I mode. If not issued, the radio will
     *         remain in STANDBY-II mode which, per the data sheet, is not a recommended
     *         operating mode.
     *
     * @note When transmitting data in rapid succession, it is still recommended by
     *       the manufacturer to drop the radio out of TX or STANDBY-II mode if there is
     *       time enough between sends for the FIFOs to empty. This is not required if auto-ack
     *       is enabled.
     *
     * @return True if transmission is successful
     */
    bool txStandBy(void);

    /**
     * @brief This function allows extended blocking and auto-retries per a user defined timeout
     *
     * @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
     *
     * @param[in] timeout Number of milliseconds to retry failed payloads
     *
     * @return True if transmission is successful
     */
    bool txStandBy(uint32_t timeout, bool startTx = 0);

    /**
     * @brief Write an ack payload for the specified pipe
     *
     * @detail The next time a message is received on @p pipe, the data in @p buf will
     *         be sent back in the acknowledgement.
     *
     * @warning Only three of these can be pending at any time as there are only 3 FIFO buffers.<br> Dynamic payloads must be enabled.
     *
     * @note Ack payloads are handled automatically by the radio chip when a payload is received. Users should generally
     *       write an ack payload as soon as startListening() is called, so one is available when a regular payload is received.
     * @note Ack payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
     *       enableDynamicPayloads() to enable on all pipes.
     *
     * @param[in] pipe      Which pipe# (typically 1-5) will get this response.
     * @param[in] p_buffer  Pointer to data that is sent
     * @param[in] length    Length of the data to send, up to 32 bytes max.  Not affected
     *                      by the static payload set by setPayloadSize().
     */
    void writeAckPayload(uint8_t pipe, const void* p_buffer, uint8_t length);

    /**
     * @brief Determine if an ack payload was received in the most recent call to
     *        write(). The regular available() can also be used.
     *
     * @detail Call read() to retrieve the ack payload.
     *
     * @return True if an ack payload is available.
     */
    bool isAckPayloadAvailable(void);

    /**
     * @brief Call this when you get an interrupt to find out why
     *
     * @detail Tells you what caused the interrupt, and clears the state of
     *         interrupts.
     *
     * @param[out] tx_ok    The send was successful (TX_DS)
     * @param[out] tx_fail  The send failed, too many retries (MAX_RT)
     * @param[out] rx_ready There is a message waiting to be read (RX_DS)
     */
    void whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready);


    /**
     * @brief Non-blocking write to the open writing pipe used for buffered writes
     *
     * @note Optimization: This function now leaves the CE pin high, so the radio
     *       will remain in TX or STANDBY-II Mode until a txStandBy() command is issued. Can be used as an alternative to startWrite()
     *       if writing multiple payloads at once.
     *
     * @warning It is important to never keep the nRF24L01 in TX mode with FIFO full for more than 4ms at a time. If the auto
     *          retransmit/autoAck is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
     *          to clear by issuing txStandBy() or ensure appropriate time between transmissions.
     *
     * @param[in] p_buffer  Pointer to the data to be sent
     * @param[in] length    Number of bytes to be sent
     * @param[in] multicast Request ACK (0) or NOACK (1)
     *
     * @return True if the payload was delivered successfully false if not
     */
    void startFastWrite(const void* p_buffer, uint8_t length, const bool multicast, bool startTx = 1);

    /**
     * @brief Non-blocking write to the open writing pipe
     *
     * @detail Just like write(), but it returns immediately. To find out what happened
     *         to the send, catch the IRQ and then call whatHappened().
     *
     * @param[in] p_buffer  Pointer to the data to be sent
     * @param[in] length    Number of bytes to be sent
     * @param[in] multicast Request ACK (0) or NOACK (1)
     */
    void startWrite(const void* p_buffer, uint8_t length, const bool multicast);

    /**
     * @brief This function is mainly used internally to take advantage of the auto payload
     *        re-use functionality of the chip, but can be beneficial to users as well.
     *
     * @detail The function will instruct the radio to re-use the data in the FIFO buffers,
     *         and instructs the radio to re-send once the timeout limit has been reached.
     *         Used by writeFast and writeBlocking to initiate retries when a TX failure
     *         occurs. Retries are automatically initiated except with the standard write().
     *         This way, data is not flushed from the buffer until switching between modes.
     *
     * @note This is to be used AFTER auto-retry fails if wanting to resend
     *       using the built-in payload reuse features.
     *       After issuing reUseTX(), it will keep reending the same payload forever or until
     *       a payload is written to the FIFO, or a flush_tx command is given.
     */
    void reUseTX(void);

    /**
     * @brief Empty the transmit buffer.
     *
     * @detail This is generally not required in standard operation.
     *         May be required in specific cases after stopListening() , if operating at 250KBPS data rate.
     *
     * @return Current value of status register
     */
    uint8_t flush_tx(void);

    /**
     * @brief Empty the receive buffer.
     *
     * @return Current value of status register
     */
    uint8_t flush_rx(void);

    /**
     * @brief Test whether there was a carrier on the line for the previous listening period.
     *
     * @detail Useful to check for interference on the current channel.
     *
     * @return true if was carrier, false if not
     */
    bool testCarrier(void);

    /**
     * @brief Test a signal (carrier or otherwise) power.
     *
     * @detail Useful to check for interference on the current channel and channel hopping strategies.
     *
     * @return Received power level in dBm.
     */
    int8_t testRPD(void);


     /**
     * @brief Close a pipe after it has been previously opened.
     *
     * @detail Can be safely called without having previously opened a pipe.
     *
     * @param[in] pipe Which pipe # to close, 0-5.
     */
    void closeReadingPipe(uint8_t pipe);

    /**@}*/
    /**
     * @name Optional Configurators
     *
     *  Methods you can use to get or set the configuration of the chip.
     *  None are required.  Calling begin() sets up a reasonable set of
     *  defaults.
     */
    /**@{*/

    /**
    * @brief Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
    *
    * @param[in] a_width The address width to use: 3,4 or 5
    */
    void setAddressWidth(uint8_t a_width);

    /**
     * @brief Set the number and delay of retries upon failed submit
     *
     * @param[in] delay How long to wait between each retry, in multiples of 250us,
     *                  max is 15.  0 means 250us, 15 means 4000us.
     * @param[in] count How many retries before giving up, max 15
     */
    void setRetries(uint8_t delay, uint8_t count);

    /**
     * @brief Set RF communication channel
     *
     * @param[in] channel Which RF channel to communicate on, 0-125
     */
    void setChannel(uint8_t channel);

    /**
     * @brief Get RF communication channel
     *
     * @return The currently configured RF Channel
     */
    uint8_t getChannel(void);

    /**
     * @brief Set Static Payload Size
     *
     * @detail This implementation uses a pre-stablished fixed payload size for all
     *         transmissions.  If this method is never called, the driver will always
     *         transmit the maximum payload size (32 bytes), no matter how much
     *         was sent to write().
     *
     * @todo Implement variable-sized payloads feature
     *
     * @param[in] size The number of bytes in the payload
     */
    void setPayloadSize(uint8_t size);

    /**
     * @brief Get Static Payload Size
     *
     * @return The number of bytes in the payload
     */
    uint8_t getPayloadSize(void);

    /**
     * @brief Get Dynamic Payload Size
     *
     * @detail For dynamic payloads, this pulls the size of the payload off the chip
     *
     * @note Corrupt packets are now detected and flushed per the manufacturer.
     *
     * @return Payload length of last-received dynamic payload
     */
    uint8_t getDynamicPayloadSize(void);

    /**
     * @brief Enable custom payloads on the acknowledge packets
     *
     * @detail Ack payloads are a handy way to return data back to senders without
     *         manually changing the radio modes on both units.
     *
     * @note Ack payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
     *       enableDynamicPayloads() to enable on all pipes.
     */
    void enableAckPayload(void);

    /**
     * @brief Enable dynamically-sized payloads
     *
     * @detail This way you don't always have to send large packets just to send them
     *         once in a while.  This enables dynamic payloads on ALL pipes.
     */
    void enableDynamicPayloads(void);

    /**
     * @brief Disable dynamically-sized payloads
     *
     * @detail This disables dynamic payloads on ALL pipes. Since Ack Payloads
     *         requires Dynamic Payloads, Ack Payloads are also disabled.
     *         If dynamic payloads are later re-enabled and ack payloads are desired
     *         then enableAckPayload() must be called again as well.
     */
    void disableDynamicPayloads(void);


    /**
     * @brief Enable dynamic ACKs (single write multicast or unicast) for chosen messages
     *
     * @note To enable full multicast or per-pipe multicast, use setAutoAck()
     *
     * @warning This MUST be called prior to attempting single write NOACK calls
     */
    void enableDynamicAck(void);

    /**
     * @brief Enable or disable auto-acknowlede packets
     *
     * @detail This is enabled by default, so it's only needed if you want to turn it off for some
     *         reason.
     *
     * @param[in] enable Whether to enable (true) or disable (false) auto-acks
     */
    void setAutoAck(bool enable);

    /**
     * @brief Enable or disable auto-acknowlede packets on a per pipeline basis.
     *
     * @detail AA is enabled by default, so it's only needed if you want to turn
     *         it off/on for some reason on a per pipeline basis.
     *
     * @param[in] pipe   Which pipeline to modify
     * @param[in] enable Whether to enable (true) or disable (false) auto-acks
     */
    void setAutoAck(uint8_t pipe, bool enable);

    /**
     * @brief Set Power Amplifier (PA) level to one of four levels:
     *        RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
     *
     * @detail The power levels correspond to the following output levels respectively:
     *         NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
     *         SI24R1: -6dBm, 0dBm, 3dBM, and 7dBm.
     *
     * @param[in] level Desired PA level.
     */
    void setPALevel(uint8_t level);

    /**
     * @brief Fetches the current PA level.
     *
     * @detail NRF24L01: -18dBm, -12dBm, -6dBm and 0dBm
     *         SI24R1:   -6dBm, 0dBm, 3dBm, 7dBm
     *
     * @return Returns values 0 to 3 representing the PA Level.
     */
     uint8_t getPALevel(void);

    /**
     * @brief Set the transmission data rate
     *
     * @warning setting RF24_250KBPS will fail for non-plus units
     *
     * @param[in] speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
     *
     * @return true if the change was successful
     */
    bool setDataRate(rf24_datarate_e speed);

    /**
     * @brief Fetches the transmission data rate
     *
     * @return Returns the hardware's currently configured datarate. The value
     *         is one of 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS, as defined in the
     *         rf24_datarate_e enum.
     */
    rf24_datarate_e getDataRate(void);

    /**
     * @brief Set the CRC length
     *
     * @detail CRC checking cannot be disabled if auto-ack is enabled
     *
     * @param[in] length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
     */
    void setCRCLength(rf24_crclength_e length);

    /**
     * @brief Get the CRC length
     *
     * @detail CRC checking cannot be disabled if auto-ack is enabled
     *
     * @return RF24_CRC_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
     */
    rf24_crclength_e getCRCLength(void);

    /**
     * @brief Disable CRC validation
     *
     * @warning CRC cannot be disabled if auto-ack/ESB is enabled.
     */
    void disableCRC(void);

    /**
    * @brief Sets possible radio external interrupt reasons.
    *
    * @detail The radio will generate interrupt signals when a transmission is complete,
    *         a transmission fails, or a payload is received. This allows users to mask
    *         those interrupts to prevent them from generating a signal on the interrupt
    *         pin. Interrupts are enabled on the radio chip by default.
    *
    * @param[in] tx_ok    Mask transmission complete interrupts
    * @param[in] tx_fail  Mask transmit failure interrupts
    * @param[in] rx_ready Mask payload received interrupts
    */
    void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);

    /**@}*/

private:
    /**
     * @name Low-level internal interface.
     *
     *  Protected methods that address the chip directly.  Regular users cannot
     *  ever call these.  They are documented for completeness and for developers who
     *  may want to extend this class.
     */
    /**@{*/

    /**
     * @brief Set chip select pin
     *
     * @detail Running SPI bus at PI_CLOCK_DIV2 so we don't waste time transferring data
     *         and best of all, we make use of the radio's FIFO buffers. A lower speed
     *         means we're less likely to effectively leverage our FIFOs and pay a higher
     *         AVR runtime cost as toll.
     *
     * @param[in] mode HIGH to take this unit off the SPI bus, LOW to put it on
     */
    void csn(bool mode);

    /**
     * @brief Set chip enable
     *
     * @param[in] level HIGH to actively begin transmission or LOW to put in standby.  Please see data sheet
     *                  for a much more detailed description of this pin.
     */
    void ce(bool level);

    /**
     * @brief Read a chunk of data in from a register
     *
     * @param[in] reg      Which register. Use constants from nRF24L01.h
     * @param[in] p_buffer Where to put the data
     * @param[in] length   How many bytes of data to transfer
     *
     * @return Current value of status register
     */
    uint8_t read_register(uint8_t reg, uint8_t* p_buffer, uint8_t length);

    /**
     * @brief Read single byte from a register
     *
     * @param[in] reg Which register. Use constants from nRF24L01.h
     *
     * @return Current value of register @p reg
     */
    uint8_t read_register(uint8_t reg);

    /**
     * @brief Write a chunk of data to a register
     *
     * @param[in] reg      Which register. Use constants from nRF24L01.h
     * @param[in] p_buffer Where to get the data
     * @param[in] length   How many bytes of data to transfer
     *
     * @return Current value of status register
     */
    uint8_t write_register(uint8_t reg, const uint8_t* p_buffer, uint8_t length);

    /**
     * @brief Write a single byte to a register
     *
     * @param[in] reg Which register. Use constants from nRF24L01.h
     * @param[in] value The new value to write
     *
     * @return Current value of status register
     */
    uint8_t write_register(uint8_t reg, uint8_t value);

    /**
     * @brief Write the transmit payload
     *
     * @detail The size of data written is the fixed payload size, see getPayloadSize()
     *
     * @param[in] p_buffer Where to get the data
     * @param[in] length   Number of bytes to be sent
     *
     * @return Current value of status register
     */
    uint8_t write_payload(const void* p_buffer, uint8_t length, const uint8_t writeType);

    /**
     * @brief Read the receive payload
     *
     * @dtail The size of data read is the fixed payload size, see getPayloadSize()
     *
     * @param[in] p_buffer Where to put the data
     * @param[in] length   Maximum number of bytes to read
     *
     * @return Current value of status register
     */
    uint8_t read_payload(void* p_buffer, uint8_t length);

    /**
     * @brief Retrieve the current status of the chip
     *
     * @return Current value of status register
     */
    uint8_t get_status(void);

    #if !defined (MINIMAL)
    /**
     * @brief Decode and print the given status to stdout
     *
     * @param[in] status Status value to print
     *
     * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
     */
    void print_status(uint8_t status);

    /**
     * @brief Decode and print the given 'observe_tx' value to stdout
     *
     * @param[in] value The observe_tx value to print
     *
     * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
     */
    void print_observe_tx(uint8_t value);

    /**
     * @brief Print the name and value of an 8-bit register to stdout
     *
     * @detail Optionally it can print some quantity of successive
     *         registers on the same line.  This is useful for printing a group
     *         of related registers on one line.
     *
     * @param[in] name Name of the register
     * @param[in] reg  Which register. Use constants from nRF24L01.h
     * @param[in] qty  How many successive registers to print
     */
    void print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);

    /**
     * @brief Print the name and value of a 40-bit address register to stdout
     *
     * @detail Optionally it can print some quantity of successive
     *         registers on the same line.  This is useful for printing a group
     *         of related registers on one line.
     *
     * @param[in] name Name of the register
     * @param[in] reg  Which register. Use constants from nRF24L01.h
     * @param[in] qty  How many successive registers to print
     */
    void print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);
    #endif

    /**
     * @brief Turn on or off the special features of the chip
     *
     * @detail The chip has certain 'features' which are only available when the 'features'
     *         are enabled.  See the datasheet for details.
     */
    void toggle_features(void);

    /**
     * @brief Built in spi transfer function to simplify code repeating code
     */
    uint8_t spiTrans(uint8_t cmd);

    /**@}*/
};

#endif // SE8R01_H__

/** @} */
