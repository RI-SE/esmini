#pragma once

#include <memory>
#include <string>
#include <vector>
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneSpeed {
   public:
	LaneSpeed(double sOffset, double max, std::string unit) : s_offset_(sOffset), max_(max), unit_(unit){};

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& lane);
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	double s_offset_;
	double max_;
	std::string unit_;
};
