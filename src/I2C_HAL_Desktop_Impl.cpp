#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <unistd.h>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>

extern "C" {
#include "I2C_HAL.h"
#include "I2C_HAL_DESKTOP.h"
}

// TODO: Read this from the environment
#define OUTPUT_FILE "output/pin_states.json"

std::map<std::string, HAL_Pin*> pinNameMap;

void HAL_registerPin(HAL_Pin* signal, const char* name) {
  std::cout << "Registering pin " << name << '\n';

  pinNameMap[name] = signal;
}

std::string getCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
  auto millis_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) - seconds_since_epoch;

  std::time_t current_time = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(std::gmtime(&current_time), "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << millis_since_epoch.count();
  return ss.str();
}

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint8_t pinNumber){
  newPin->pinNumber = pinNumber;
  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  printf("Selecting pin %d direction: %s\n", pin->pinNumber, direction == INPUT ? "INPUT" : "OUTPUT");
  pin->direction = direction;
}

std::string getLastLineFromFile(const std::string& filename){
  std::ifstream ifs(OUTPUT_FILE);
  std::string lastLine;
  if (ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      lastLine = line;
    }
    ifs.close();
  }

  return lastLine;
}

// Parse the last line by storing the pin states in a name-value map, timestamp and combined I2C output in variables
// The example last line is: {"timestamp":"19:18:58.646","signals":[{"MASTER_SCL_OUT":0},{"MASTER_SDA_OUT":0},{"SLAVE_SCL_OUT":0},{"SLAVE_SDA_OUT":0}],"SCL":1,"SDA":1}
void parsePinStatesFromJSON(const std::string& jsonString, std::string& timestamp, std::map<std::string, int>& pinStates, int& scl, int& sda){
  if (jsonString.size() > 0) {
    rapidjson::Document document;
    document.Parse(jsonString.c_str());
    timestamp = document["timestamp"].GetString();
    for (auto& signal : document["signals"].GetArray()) {
      pinStates[signal.GetObject().MemberBegin()->name.GetString()] = signal.GetObject().MemberBegin()->value.GetInt();
    }

    if (document.HasMember("SCL")) {
      scl = document["SCL"].GetInt();
    } else {
      scl = 0;
    }

    if (document.HasMember("SDA")) {
      sda = document["SDA"].GetInt();
    } else {
      sda = 0;
    }
  } else {
    std::cout << "JSON string is empty\n";
    scl = 0;
    sda = 0;
  }
}

rapidjson::Document constructJSONFromPinStates(std::map<std::string, int>& pinStates, int scl, int sda){
  rapidjson::Document document;
  document.SetObject();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

  rapidjson::Value timestampValue(getCurrentTimestamp().c_str(), allocator);
  document.AddMember("timestamp", timestampValue, allocator);

  // Create a new signals array with all the output pins from the pinStates map and update only those pins that are in the pins map
  rapidjson::Value newSignals(rapidjson::kArrayType);

  for(auto& [name, pin] : pinNameMap){
    if(name.find("OUT") == std::string::npos){
      continue;
    }

    rapidjson::Value signal(rapidjson::kObjectType);
    rapidjson::Value nameValue(name.c_str(), allocator);

    // Update the pin state
    rapidjson::Value pinValue(pin->level);
    pinStates[name] = pin->level;
    signal.AddMember(nameValue, pinValue, allocator);

    newSignals.PushBack(signal, allocator);
  }

  // Copy the remaining pins from the pinStates map
  for(auto& [name, pin] : pinStates){
    if(pinNameMap.find(name) == pinNameMap.end()){
      rapidjson::Value signal(rapidjson::kObjectType);
      rapidjson::Value nameValue(name.c_str(), allocator);
      rapidjson::Value pinValue(pin);
      signal.AddMember(nameValue, pinValue, allocator);

      newSignals.PushBack(signal, allocator);
    }
  }

  document.AddMember("signals", newSignals, allocator);

  // Update the SCL and SDA values after calculating them from the pins containing their names from updated pins map
  // If one pin is HIGH, the value is LOW, if both pins are LOW, the value is LOW, otherwise the value is HIGH
  std::map<std::string, int> sclPinStates;
  std::map<std::string, int> sdaPinStates;

  scl = 1;
  sda = 1;

  for (auto& [name, pin] : pinStates) {

    if (name.find("SCL") != std::string::npos) {
      sclPinStates[name] = pin;
    } else if (name.find("SDA") != std::string::npos) {
      sdaPinStates[name] = pin;
    }
  }

  for (auto& [name, pin] : sclPinStates) {
    if (pin == 1) {
      scl = 0;
      break;
    }
  }

  for (auto& [name, pin] : sdaPinStates) {
    if (pin == 1) {
      sda = 0;
      break;
    }
  }

  rapidjson::Value sclValue(scl);
  document.AddMember("SCL", sclValue, allocator);

  rapidjson::Value sdaValue(sda);
  document.AddMember("SDA", sdaValue, allocator);

  return document;
}

void appendToFile(const std::string& filename, const rapidjson::Document& data){
  std::ofstream ofs;
  ofs.open(OUTPUT_FILE, std::ofstream::out | std::ofstream::app);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
  data.Accept(writer);
  ofs << '\n';
  ofs.close();
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level) {
  printf("Writing to pin %d: %d\n", pin->pinNumber, level);
  pin->level = level;

  // Get last line of the file
  std::string lastLine = getLastLineFromFile(OUTPUT_FILE);

  // Parse pin states from JSON
  std::string timestamp;
  std::map<std::string, int> pinStates;
  int scl, sda;
  parsePinStatesFromJSON(lastLine, timestamp, pinStates, scl, sda);

  // Construct new JSON document
  rapidjson::Document newDocument = constructJSONFromPinStates(pinStates, scl, sda);

  // Append new JSON to the file
  appendToFile(OUTPUT_FILE, newDocument);
}

// Read the last line from the file and parse the pin states from it
HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  if(pin->direction == OUTPUT){
    std::cout << "WARNING: Reading from an output pin. Returning LOW.\n";
    return HAL_PinLevel::LOW;
  }

  // Get last line of the file
  std::string lastLine = getLastLineFromFile(OUTPUT_FILE);

  // Parse pin states from JSON
  std::string timestamp;
  std::map<std::string, int> pinStates;
  int scl, sda;
  parsePinStatesFromJSON(lastLine, timestamp, pinStates, scl, sda);

  // Find the pin name in the pins map and then find it's value in the pinStates map
  for(auto& [name, pinPointer] : pinNameMap){
    if(pinPointer == pin){
      if(name.find("SCL") != std::string::npos){
        pin->level = scl == 1 ? HAL_PinLevel::HIGH : HAL_PinLevel::LOW;
      } else if(name.find("SDA") != std::string::npos){
        pin->level = sda == 1 ? HAL_PinLevel::HIGH : HAL_PinLevel::LOW;
      }
      
      return pin->level;
      break;
    }
  }

  std::cout << "WARNING: Pin not found in the map. Returning LOW.\n";

  return HAL_PinLevel::LOW;
}

void HAL_sleep(uint16_t ms){
  usleep(ms * 1000);
}