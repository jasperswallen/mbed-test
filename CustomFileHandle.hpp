#include <mbed.h>

#include <vector>

class CustomFileHandle : public FileHandle
{
public:
    CustomFileHandle()
    {
    }

    virtual ssize_t write(const void *buffer, size_t size) final
    {
        internal_buf.emplace_back(
            reinterpret_cast<const char *>(buffer),
            reinterpret_cast<const char *>(buffer) + size);
        internal_buf.back().push_back('\0');

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

    const std::vector<std::vector<char>> &get_internal_buf()
    {
        return internal_buf;
    }

private:
    std::vector<std::vector<char>> internal_buf;
};
