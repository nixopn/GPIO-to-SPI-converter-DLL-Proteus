#pragma once
#include "sdk/vsm.hpp"

namespace details {
    const RELTIME DELAY = (RELTIME) 1;
}

class GPIO_SPI_Converter : public IDSIMMODEL {
private:
    IINSTANCE* m_instance;
    IDSIMCKT* m_ckt;
    ABSTIME m_currentTime;
    BOOL m_firstSPIClock;
    BYTE m_byteCount;
    BYTE m_multiByte;
    BYTE m_originalMultiByte;


    IDSIMPIN2* m_DI[8];
    IDSIMPIN2* m_DO[8];
    IDSIMPIN2* m_CLK;
    IDSIMPIN2* m_MODE;
    IDSIMPIN2* m_MOSI_STROBE;
    IDSIMPIN2* m_CS;
    IDSIMPIN2* m_RST;
    IDSIMPIN2* m_INT;


    IDSIMPIN2* m_SPI_SCLK;
    IDSIMPIN2* m_SPI_MOSI;
    IDSIMPIN2* m_SPI_MISO;
    //IDSIMPIN2* m_SPI_CS;


    BYTE m_txData;
    BYTE m_rxData;
    BOOL m_rxReady;


    BOOL m_spiEnable;
    BOOL m_intEnable;
    BYTE m_spiClkDiv;
    BYTE m_spiMode;
    BYTE m_spiClkCounter;


    BYTE m_shiftReg;
    BYTE m_originalTxData;
    BYTE m_bitCount;
    BOOL m_spiBusy;
    BOOL m_prevSPIClock;


    BOOL m_prevCLK;


    BYTE readDI();
    VOID writeDO(BYTE data);
    VOID startSPI();
    VOID handleSPI();
    VOID resetDevice();
    VOID processConfig(BYTE addr, BYTE data);
    BYTE readConfig(BYTE addr);


    IDSIMPIN2* m_A0;
    IDSIMPIN2* m_A1;
    IDSIMPIN2* m_A2;


    IDSIMPIN2* m_SPI_CS[8];
    BYTE m_csConfig;

    BYTE m_deviceAddr;
    BYTE m_myAddr;


public:
    static constexpr DWORD MODEL_KEY = 0x3A4B5C6D;

    GPIO_SPI_Converter();
    virtual ~GPIO_SPI_Converter();

    INT isdigital(CHAR* pinname) override;
    VOID setup(IINSTANCE* instance, IDSIMCKT* dsim) override;
    VOID runctrl(RUNMODES mode) override;
    VOID actuate(REALTIME time, ACTIVESTATE newstate) override;
    BOOL indicate(REALTIME time, ACTIVEDATA* newstate) override;
    VOID simulate(ABSTIME time, DSIMMODES mode) override;
    VOID callback(ABSTIME time, EVENTID eventid) override;
    VOID updateSPICS();
    BYTE countActiveCS();
};
