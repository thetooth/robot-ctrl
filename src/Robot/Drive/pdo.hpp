#ifndef FSM_PDO_HPP
#define FSM_PDO_HPP

#include <cstdint>

namespace Drive
{
    class PDO
    {
      public:
        virtual ~PDO() = default;

        virtual uint16_t getStatusWord() const = 0;
        virtual int32_t getActualPosition() const = 0;
        virtual int32_t getActualVelocity() const = 0;
        virtual int16_t getActualTorque() const = 0;
        virtual int32_t getFollowingError() const = 0;
        virtual uint16_t getErrorCode() const = 0;
        virtual uint32_t getDigitalInputs() const = 0;
        virtual bool getEmergencyStop() const = 0;

        virtual void setControlWord(uint16_t value) = 0;
        virtual void setTargetPosition(int32_t value) = 0;
        virtual void setTargetVelocity(int32_t value) = 0;
        virtual void setTargetTorque(int16_t value) = 0;
        virtual void setDigitalOutputs(uint32_t value) = 0;
    };
} // namespace Drive

#endif
