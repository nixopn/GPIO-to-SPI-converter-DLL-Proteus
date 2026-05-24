#include "pch.h"
#include "GPIO_SPI_Converter.h"
#include <cstdio>

GPIO_SPI_Converter::GPIO_SPI_Converter()
    : m_instance(nullptr), m_ckt(nullptr), m_CLK(nullptr), m_MODE(nullptr), m_MOSI_STROBE(nullptr), m_CS(nullptr),
      m_RST(nullptr), m_INT(nullptr), m_SPI_SCLK(nullptr), m_SPI_MOSI(nullptr), m_SPI_MISO(nullptr), m_SPI_CS(nullptr),
      m_txData(0), m_rxData(0), m_rxReady(FALSE), m_spiEnable(TRUE), m_intEnable(FALSE), m_spiClkDiv(0), m_spiMode(0),
      m_spiClkCounter(0), m_shiftReg(0), m_bitCount(0), m_spiBusy(FALSE), m_prevSPIClock(FALSE), m_prevCLK(FALSE),
      m_currentTime(0), m_byteCount(0), m_multiByte(0) {
    m_originalTxData = 0;
    for (int i = 0; i < 8; i++) {
        m_DI[i] = nullptr;
        m_DO[i] = nullptr;
    }
}

GPIO_SPI_Converter::~GPIO_SPI_Converter() {}

INT GPIO_SPI_Converter::isdigital(CHAR* pinname) {
    return TRUE;
}

VOID GPIO_SPI_Converter::setup(IINSTANCE* instance, IDSIMCKT* dsim) {
    m_instance = instance;
    m_ckt = dsim;

    CHAR name[5];
    for (int i = 0; i < 8; i++) {
        sprintf(name, "D%d", i);
        m_DI[i] = (IDSIMPIN2*) instance->getdsimpin(name, TRUE);
        sprintf(name, "D%d", i + 12);
        m_DO[i] = (IDSIMPIN2*) instance->getdsimpin(name, TRUE);
        if (m_DO[i]) {
            m_DO[i]->settiming(details::DELAY, details::DELAY, details::DELAY);
            m_DO[i]->setstates(SHI, SLO, FLT);
            m_DO[i]->setstate(FLT);
        }
    }

    char CLK[] = "CLK";
    char MODE[] = "MODE";
    char STROBE[] = "MOSI_STROBE";
    char CS[] = "CS";
    char RST[] = "RST";
    char INT[] = "INT";
    m_CLK = (IDSIMPIN2*) instance->getdsimpin(CLK, TRUE);
    m_MODE = (IDSIMPIN2*) instance->getdsimpin(MODE, TRUE);
    m_MOSI_STROBE = (IDSIMPIN2*) instance->getdsimpin(STROBE, TRUE);
    m_CS = (IDSIMPIN2*) instance->getdsimpin(CS, TRUE);
    m_RST = (IDSIMPIN2*) instance->getdsimpin(RST, TRUE);
    m_INT = (IDSIMPIN2*) instance->getdsimpin(INT, TRUE);

    CHAR spi_sclk[] = "SPI_SCLK";
    CHAR spi_mosi[] = "SPI_MOSI";
    CHAR spi_miso[] = "SPI_MISO";
    CHAR spi_cs[] = "SPI_CS";
    m_SPI_SCLK = (IDSIMPIN2*) instance->getdsimpin(spi_sclk, TRUE);
    m_SPI_MOSI = (IDSIMPIN2*) instance->getdsimpin(spi_mosi, TRUE);
    m_SPI_MISO = (IDSIMPIN2*) instance->getdsimpin(spi_miso, TRUE);
    m_SPI_CS = (IDSIMPIN2*) instance->getdsimpin(spi_cs, TRUE);

    if (m_SPI_SCLK) {
        m_SPI_SCLK->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_SPI_SCLK->setstates(SHI, SLO, FLT);
        m_SPI_SCLK->setstate(SLO);
    }
    if (m_SPI_MOSI) {
        m_SPI_MOSI->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_SPI_MOSI->setstates(SHI, SLO, FLT);
        m_SPI_MOSI->setstate(SLO);
    }
    if (m_SPI_CS) {
        m_SPI_CS->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_SPI_CS->setstates(SHI, SLO, FLT);
        m_SPI_CS->setstate(SHI);
    }
    if (m_INT) {
        m_INT->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_INT->setstates(SHI, SLO, FLT);
        m_INT->setstate(SLO);
    }

    CHAR msg[] = "SETUP COMPLETE";
    m_instance->log(msg);
}

VOID GPIO_SPI_Converter::runctrl(RUNMODES mode) {
    /*if (mode == RM_STOP || mode == RM_SUSPEND) resetDevice();*/
}

VOID GPIO_SPI_Converter::actuate(REALTIME time, ACTIVESTATE newstate) {}
BOOL GPIO_SPI_Converter::indicate(REALTIME time, ACTIVEDATA* newstate) {
    return FALSE;
}

VOID GPIO_SPI_Converter::simulate(ABSTIME time, DSIMMODES mode) {
    m_currentTime = time;

    if (!m_CLK || !m_CS || !m_MODE || !m_MOSI_STROBE) return;

    STATE cs = m_CS->istate();
    if (ishigh(cs)) {
        m_prevCLK = ishigh(m_CLK->istate());
        return;
    }

    STATE clkState = m_CLK->istate();
    BOOL clkHigh = ishigh(clkState);

    if (clkHigh && !m_prevCLK) {
        STATE modeState = m_MODE->istate();
        BOOL configMode = ishigh(modeState);
        BOOL strobeActive = ishigh(m_MOSI_STROBE->istate());

        if (configMode) {
            BYTE input = readDI();
            BYTE addr = input & 0x0F;
            if (strobeActive) {
                BYTE data = (input >> 4) & 0x0F;
                processConfig(addr, data);
            } else {
                BYTE cfgData = readConfig(addr);
                writeDO(cfgData);
            }
        } else if (strobeActive && !m_spiBusy) {
            m_txData = readDI();
            m_rxReady = FALSE;
            startSPI();
        }
    }

    m_prevCLK = clkHigh;

    if (m_spiBusy && m_spiEnable) {
        m_spiClkCounter++;
        if (m_spiClkCounter >= m_spiClkDiv) {
            m_spiClkCounter = 0;
            handleSPI();
        }
    } else if (!m_spiBusy && m_prevSPIClock) {
        if (m_SPI_CS) m_SPI_CS->setstate(m_currentTime, details::DELAY, SHI);
        if (m_SPI_SCLK) m_SPI_SCLK->setstate(m_currentTime, details::DELAY, SLO);
        m_prevSPIClock = FALSE;
    }

    if (m_INT) {
        BOOL intActive = m_rxReady && m_intEnable;
        m_INT->setstate(m_currentTime, details::DELAY, intActive ? SHI : SLO);
    }
}



VOID GPIO_SPI_Converter::callback(ABSTIME time, EVENTID eventid) {}

VOID GPIO_SPI_Converter::resetDevice() {
    CHAR msg[] = "RESET DEVICE";
    m_instance->log(msg);
    m_spiEnable = TRUE;
    m_intEnable = FALSE;
    m_spiClkDiv = 0;
    m_spiMode = 0;
    m_spiBusy = FALSE;
    m_bitCount = 0;
    m_rxReady = FALSE;
    m_txData = 0;
    m_rxData = 0;
    m_spiClkCounter = 0;
    if (m_SPI_CS) m_SPI_CS->setstate(m_currentTime, details::DELAY, SHI);
    if (m_SPI_SCLK) m_SPI_SCLK->setstate(m_currentTime, details::DELAY, SLO);
    if (m_INT) m_INT->setstate(m_currentTime, details::DELAY, SLO);
    writeDO(0);
}

VOID GPIO_SPI_Converter::processConfig(BYTE addr, BYTE data) {
    CHAR buf[64];
    sprintf(buf, "processConfig: addr=%d data=%d", addr, data);
    m_instance->log(buf);
    switch (addr) {
        case 0x00:
            m_spiEnable = data & 0x01;
            m_intEnable = data & 0x02;
            break;
        case 0x01:
            m_spiClkDiv = data & 0x0F;
            break;
        case 0x02:
            m_spiMode = data & 0x03;
            break;
        case 0x03:
            m_multiByte = data & 0x03;
            break;
    }
}

BYTE GPIO_SPI_Converter::readConfig(BYTE addr) {
    switch (addr) {
        case 0x00:
            return (m_spiEnable ? 0x01 : 0) | (m_intEnable ? 0x02 : 0);
        case 0x01:
            return m_spiClkDiv;
        case 0x02:
            return m_spiMode;
        case 0x03:
            return m_multiByte & 0x03;
        default:
            return 0;
    }
}

BYTE GPIO_SPI_Converter::readDI() {
    BYTE data = 0;
    for (int i = 0; i < 8; i++) {
        if (m_DI[i] && ishigh(m_DI[i]->istate())) data |= (1 << (7 - i));
    }
    CHAR buf[32];
    sprintf(buf, "readDI=0x%02X", data);
    //m_instance->log(buf);
    return data;
}

VOID GPIO_SPI_Converter::writeDO(BYTE data) {
    CHAR buf[32];
    sprintf(buf, "writeDO=0x%02X", data);
    //m_instance->log(buf);
    for (int i = 0; i < 8; i++) {
        if (m_DO[i]) m_DO[i]->setstate(m_currentTime, details::DELAY, (data & (1 << (7 - i))) ? SHI : SLO);
    }
}

VOID GPIO_SPI_Converter::startSPI() {
    m_shiftReg = m_txData;
    m_originalTxData = m_txData;
    CHAR buf222[64];
    sprintf(buf222, "startSPI: txData=0x%02X original=0x%02X", m_txData, m_originalTxData);
    m_instance->log(buf222);
    m_rxData = 0;
    m_bitCount = 0;
    m_spiBusy = TRUE;
    m_prevSPIClock = FALSE;
    m_firstSPIClock = FALSE;
    m_spiClkCounter = 0;

    CHAR buf[64];
    sprintf(buf, "startSPI: txData=0x%02X", m_txData);
    m_instance->log(buf);

    if (m_SPI_CS) m_SPI_CS->setstate(m_currentTime, details::DELAY, SLO);
    if (m_SPI_SCLK) m_SPI_SCLK->setstate(m_currentTime, details::DELAY, SLO);
    if (m_SPI_MOSI) m_SPI_MOSI->setstate(m_currentTime, details::DELAY, (m_shiftReg >> 7) ? SHI : SLO);
}

VOID GPIO_SPI_Converter::handleSPI() {
    if (!m_SPI_SCLK || !m_SPI_MOSI || !m_SPI_MISO) return;
    if (!m_spiBusy) return;

    if (m_firstSPIClock) {
        m_firstSPIClock = FALSE;
        return;
    }

    BOOL currentClock = !m_prevSPIClock;

    if (currentClock) {
        STATE miso = m_SPI_MISO->istate();
        m_rxData = (m_rxData << 1) | (ishigh(miso) ? 1 : 0);
    } else {
        m_bitCount++;

        if (m_bitCount >= 8) {
            m_byteCount++;

            if (m_byteCount >= m_multiByte) {
                m_spiBusy = FALSE;
                m_rxReady = TRUE;
                writeDO(m_rxData);
                m_byteCount = 0;
                m_bitCount = 0;
                return;
            } else {
                m_shiftReg = m_originalTxData;
                CHAR buf[64];
                sprintf(buf, "Second byte: shiftReg=0x%02X", m_shiftReg);
                m_instance->log(buf);
                m_bitCount = 0;
            }
        } else {
            BYTE nextBit = (m_shiftReg >> (7 - m_bitCount)) & 1;
            if (m_SPI_MOSI) m_SPI_MOSI->setstate(m_currentTime, details::DELAY, nextBit ? SHI : SLO);
        }
    }

    if (m_SPI_SCLK) m_SPI_SCLK->setstate(m_currentTime, details::DELAY, currentClock ? SHI : SLO);
    m_prevSPIClock = currentClock;
    char buf[64];
    sprintf(buf, "SPI done: rxData=0x%02X", m_rxData);
    m_instance->log(buf);
}

extern "C" {
    __declspec(dllexport) IDSIMMODEL* createdsimmodel(CHAR* device, ILICENCESERVER* ils) {
        return new GPIO_SPI_Converter();
    }
    __declspec(dllexport) VOID deletedsimmodel(IDSIMMODEL* model) {
        delete (GPIO_SPI_Converter*) model;
    }
}
