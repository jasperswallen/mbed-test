#ifndef KX134I2C_H
#define KX134I2C_H

#include "KX134Base.h"

class KX134I2C : public KX134Base
{
public:
    KX134I2C(PinName sda, PinName scl, uint8_t i2c_addr_);

    /**
     * @brief Initializes the KX134
     *
     * @return true if the init is successful, false otherwise
     */
    virtual bool init() override;

protected:
    /**
     * @brief Reads a given register a given number of bytes
     *
     * @param[in] addr The register to read from
     * @param[out] rx_buf The buffer to read into
     * @param[in] size The number of bytes to read
     */
    virtual void readRegister(Register addr, char* rx_buf, int size = 1) override;

    /**
     * @brief Writes data to a given register
     *
     * @param[in] addr The register to write to
     * @param[in] data The data to write
     * @param[out] rx_buf The response to receive
     * @param[in] size The number of bytes to write.
     */
    virtual void writeRegister(Register addr, char* data, char* rx_buf = nullptr, int size = 1) override;

private:
    I2C i2c_;

    const uint8_t i2c_addr;

};

#endif
