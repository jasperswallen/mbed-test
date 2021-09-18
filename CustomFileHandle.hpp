#include <mbed.h>

class CustomFileHandle : public FileHandle
{
public:
    CustomFileHandle() : internal_buf(nullptr)
    {
    }

    virtual ssize_t write(const void *buffer, size_t size) final
    {
        /* Appends data to internal array */
        if (memcpy(internal_buf, buffer, size))
        {
            return -ENOMEM;
        }

        return size;
    }

    ssize_t read(void *buffer, size_t size) override
    {
        /* Reading is not supported by this file handle */
        return -EBADF;
    }

    off_t seek(off_t offset, int whence = SEEK_SET) override
    {
        /* Seeking is not support by this file handler */
        return -ESPIPE;
    }

    off_t size() override
    {
        /* Size is not defined for this file handle */
        return -EINVAL;
    }

    virtual int close() final
    {
        return 0;
    }

    void *get_internal_buf()
    {
        return internal_buf;
    }

private:
    void *internal_buf;
};
