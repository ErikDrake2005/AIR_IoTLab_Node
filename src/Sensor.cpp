#include "Sensor.h"

namespace {
HoldingRegisterStrategy g_holdingStrategy;
}

uint8_t HoldingRegisterStrategy::read(ModbusMaster& node, uint16_t startReg, uint16_t count) {
    return node.readHoldingRegisters(startReg, count);
}

ModbusSensorBase::ModbusSensorBase(ModbusContext& ctx, uint8_t address, IRegisterReadStrategy* strategy,
                                   uint16_t startReg, uint16_t regCount)
    : _address(address), _strategy(strategy), _startReg(startReg), _regCount(regCount) {
    ctx.attach(_node, address);
}

bool ModbusSensorBase::read(SensorReading& out) {
    out.type = type();
    out.value1 = -1.0f;
    out.value2 = -1.0f;
    out.ok = false;

    if (_strategy == nullptr) return false;

    uint8_t result = _strategy->read(_node, _startReg, _regCount);
    if (result != ModbusMaster::ku8MBSuccess) return false;

    if (!parseRegisters(out)) return false;

    out.ok = true;
    return true;
}

EPMedCO2Sensor::EPMedCO2Sensor(ModbusContext& ctx, uint8_t address)
    : ModbusSensorBase(ctx, address, &g_holdingStrategy, kRegAddress, kRegCount) {}

bool EPMedCO2Sensor::parseRegisters(SensorReading& out) {
    uint16_t raw = node().getResponseBuffer(0);
    out.value1 = static_cast<float>(raw) * kScale;
    return true;
}

EPEnvCH4Sensor::EPEnvCH4Sensor(ModbusContext& ctx, uint8_t address)
    : ModbusSensorBase(ctx, address, &g_holdingStrategy, kRegAddress, kRegCount) {}

bool EPEnvCH4Sensor::parseRegisters(SensorReading& out) {
    uint16_t raw = node().getResponseBuffer(0);
    out.value1 = static_cast<float>(raw) * kScale;
    return true;
}

EP35SWSensor::EP35SWSensor(ModbusContext& ctx, uint8_t address)
    : ModbusSensorBase(ctx, address, &g_holdingStrategy, kRegAddress, kRegCount) {}

bool EP35SWSensor::parseRegisters(SensorReading& out) {
    int16_t tempRaw = static_cast<int16_t>(node().getResponseBuffer(0));
    int16_t humRaw = static_cast<int16_t>(node().getResponseBuffer(1));

    out.value1 = static_cast<float>(tempRaw) * kTempScale;
    out.value2 = static_cast<float>(humRaw) * kHumScale;
    return true;
}

std::unique_ptr<ISensor> SensorFactory::create(SensorType type, uint8_t address, ModbusContext& ctx) {
    switch (type) {
        case SensorType::Co2:
            return std::unique_ptr<ISensor>(new EPMedCO2Sensor(ctx, address));
        case SensorType::Ch4:
            return std::unique_ptr<ISensor>(new EPEnvCH4Sensor(ctx, address));
        case SensorType::TempHum:
            return std::unique_ptr<ISensor>(new EP35SWSensor(ctx, address));
        default:
            return nullptr;
    }
}

bool SensorRegistry::addOrUpdate(std::unique_ptr<ISensor> sensor) {
    if (!sensor) return false;

    uint8_t address = sensor->address();
    for (auto& item : _sensors) {
        if (item->address() == address) {
            item = std::move(sensor);
            return true;
        }
    }

    _sensors.push_back(std::move(sensor));
    return true;
}

bool SensorRegistry::removeByAddress(uint8_t address) {
    for (size_t i = 0; i < _sensors.size(); ++i) {
        if (_sensors[i]->address() == address) {
            _sensors.erase(_sensors.begin() + i);
            return true;
        }
    }

    return false;
}

ISensor* SensorRegistry::findByAddress(uint8_t address) const {
    for (const auto& item : _sensors) {
        if (item->address() == address) return item.get();
    }

    return nullptr;
}
