#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>
#include <memory>
#include <vector>
#include "ModbusContext.h"

enum class SensorType : uint8_t {
	Co2,
	Ch4,
	TempHum
};

struct SensorReading {
	SensorType type;
	float value1;
	float value2;
	bool ok;
};

class IRegisterReadStrategy {
public:
	virtual ~IRegisterReadStrategy() = default;
	virtual uint8_t read(ModbusMaster& node, uint16_t startReg, uint16_t count) = 0;
};

class HoldingRegisterStrategy : public IRegisterReadStrategy {
public:
	uint8_t read(ModbusMaster& node, uint16_t startReg, uint16_t count) override;
};

class ISensor {
public:
	virtual ~ISensor() = default;
	virtual uint8_t address() const = 0;
	virtual SensorType type() const = 0;
	virtual const char* name() const = 0;
	virtual bool read(SensorReading& out) = 0;
};

class ModbusSensorBase : public ISensor {
public:
	uint8_t address() const override { return _address; }
	bool read(SensorReading& out) override;

protected:
	ModbusSensorBase(ModbusContext& ctx, uint8_t address, IRegisterReadStrategy* strategy,
					 uint16_t startReg, uint16_t regCount);
	ModbusMaster& node() { return _node; }

	virtual bool parseRegisters(SensorReading& out) = 0;

private:
	ModbusMaster _node;
	uint8_t _address;
	IRegisterReadStrategy* _strategy;
	uint16_t _startReg;
	uint16_t _regCount;
};

class EPMedCO2Sensor : public ModbusSensorBase {
public:
	static constexpr uint16_t kRegAddress = 0x0000;
	static constexpr uint16_t kRegCount = 2;
	static constexpr float kMinPpm = 0.0f;
	static constexpr float kMaxPpm = 100000.0f;

	EPMedCO2Sensor(ModbusContext& ctx, uint8_t address);
	SensorType type() const override { return SensorType::Co2; }
	const char* name() const override { return "EP-MED-CO2-01"; }

protected:
	bool parseRegisters(SensorReading& out) override;
};

class EPEnvCH4Sensor : public ModbusSensorBase {
public:
	static constexpr uint16_t kRegAddress = 0x0000;
	static constexpr uint16_t kRegCount = 1;
	static constexpr float kScale = 1.0f;
	static constexpr float kMinPercentLel = 0.0f;
	static constexpr float kMaxPercentLel = 100.0f;

	EPEnvCH4Sensor(ModbusContext& ctx, uint8_t address);
	SensorType type() const override { return SensorType::Ch4; }
	const char* name() const override { return "EP-ENV-CH4-01"; }

protected:
	bool parseRegisters(SensorReading& out) override;
};

class ES35SWSensor : public ModbusSensorBase {
public:
	static constexpr uint16_t kRegAddress = 0x0000;
	static constexpr uint16_t kRegCount = 2;
	static constexpr float kTempScale = 0.1f;
	static constexpr float kHumScale = 0.1f;
	static constexpr float kMinTempC = -20.0f;
	static constexpr float kMaxTempC = 80.0f;
	static constexpr float kMinHumRh = 0.0f;
	static constexpr float kMaxHumRh = 100.0f;

	ES35SWSensor(ModbusContext& ctx, uint8_t address);
	SensorType type() const override { return SensorType::TempHum; }
	const char* name() const override { return "ES35-SW"; }

protected:
	bool parseRegisters(SensorReading& out) override;
};

class SensorFactory {
public:
	static std::unique_ptr<ISensor> create(SensorType type, uint8_t address, ModbusContext& ctx);
};

class SensorRegistry {
public:
	bool addOrUpdate(std::unique_ptr<ISensor> sensor);
	bool removeByAddress(uint8_t address);
	ISensor* findByAddress(uint8_t address) const;
	const std::vector<std::unique_ptr<ISensor>>& list() const { return _sensors; }

private:
	std::vector<std::unique_ptr<ISensor>> _sensors;
};


