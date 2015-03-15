#ifndef _EXTERNALDAC_H
#define _EXTERNALDAC_H

class ExternalDac
{
public:
    ExternalDac();
    static void begin(void);
    static void setValueAll(uint8_t value);
    static void setValue(uint8_t channel, uint8_t value);
};

#endif //_EXTERNALDAC_H

