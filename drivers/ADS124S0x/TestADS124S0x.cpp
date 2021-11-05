#include <mbed.h>

#include <ADS124S0x.h>

#include <cinttypes>

int main()
{
    PinName MOSI = PE_14;
    PinName MISO = PE_13;
    PinName SCLK = PE_12;
    PinName SSEL = PE_10;
    ADS124S0x TestADC(MOSI, MISO, SCLK, SSEL);

    while (true)
    {
        printf("\nADS124S0x Test Suite:\n");

        // TODO: Add tests

        int test = -1;

        // Menu
        printf("Select a test: \n");
        printf("1.  Test ADC init\n");
        printf("2.  Test CRC");
        printf("9.  Exit Test Suite\n");

        scanf("%d", &test);
        printf("Running test %d:\n\n", test);

        // Run Tests
        switch (test)
        {
            case 1:
                // TODO: check existence
                TestADC.init();
                break;
            case 2:
            {
                uint8_t testMessage[] = {0xa2, 0x8c, 0x64, 0x00};

                uint32_t mbedResult;
                MbedCRC<POLY_8BIT_CCITT, 8> crcCalc;
                crcCalc.compute(testMessage, sizeof(testMessage), &mbedResult);

                // expected value from here:
                // https://www.ghsi.de/pages/subpages/Online%20CRC%20Calculation/index.php?Polynom=100000111&Message=E100CAFE
                if (mbedResult == 0x83)
                {
                    printf("CRC Success!\n");
                }

                printf("Expected Value: %#04X\tMbed Value: %#04" PRIX32 "\n", 0x83, mbedResult);
                break;
            }
            case 9:
                printf("Exiting Test Suite\n");
                return 0;
            default:
                printf("Invalid Test Number Selection.\n");
        }

        printf("Done.\r\n");
    }
}
