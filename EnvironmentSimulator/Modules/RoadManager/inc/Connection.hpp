#pragma once

#include <memory>
#include <vector>
#include "Road.hpp"
#include "StructsandDefines.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"
class Road;

class JunctionLaneLink {
   public:
	JunctionLaneLink(int from, int to) : from_(from), to_(to) {}
	int from_;
	int to_;
	void Print() { printf("JunctionLaneLink: from %d to %d\n", from_, to_); }
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& connection);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
};

class Connection {
   public:
	Connection(std::shared_ptr<Road> incoming_road,
			   std::shared_ptr<Road> connecting_road,
			   ContactPointType contact_point);
	Connection(int id,
			   std::shared_ptr<Road> incoming_road,
			   std::shared_ptr<Road> connecting_road,
			   ContactPointType contact_point);
	~Connection();
	int GetNumberOfLaneLinks() { return (int)lane_link_.size(); }
	std::shared_ptr<JunctionLaneLink> GetLaneLink(int idx) { return lane_link_[idx]; }
	int GetConnectingLaneId(int incoming_lane_id);
	std::shared_ptr<Road> GetIncomingRoad() { return incoming_road_; }
	std::shared_ptr<Road> GetConnectingRoad() { return connecting_road_; }
	ContactPointType GetContactPoint() { return contact_point_; }
	void AddJunctionLaneLink(int from, int to);
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& junction);

	std::vector<std::shared_ptr<JunctionLaneLink>> getJunctionLaneLinkVector() { return lane_link_; }
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }

   protected:
	std::vector<std::shared_ptr<JunctionLaneLink>> lane_link_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	std::shared_ptr<Road> incoming_road_;
	std::shared_ptr<Road> connecting_road_;
	ContactPointType contact_point_;
	int id_;
};
