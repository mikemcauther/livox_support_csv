//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <time.h>
#include <cmath>
#include <cstring>
#include <string>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include "csv_file.h"
#include "third_party/rapidxml/rapidxml.hpp"
#include "third_party/rapidxml/rapidxml_utils.hpp"

#define WRITE_BUFFER_LEN 1024 * 1024
#define MAGIC_CODE       (0xac0ea767)
#define RAW_POINT_NUM     100
#define SINGLE_POINT_NUM  96
#define DUAL_POINT_NUM    48
#define TRIPLE_POINT_NUM  30
#define IMU_POINT_NUM     1
#define M_PI             3.14159265358979323846

#if 0
bool CSVFileDumpImpl::InitCSVFileHeader() {
    std::cout << "(InitCSVFileHeader)Not Supported Datatype = " << this->dateType << "\n";
}

void CSVFileDumpImpl::SaveEthPacketToCSVFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {
    std::cout << "(SaveEthPacketToLvxFile)Not Supported Datatype = " << dataType << "\n";
}
#endif

bool CSVFileDumpImpl::InitCSVFile() {
  csv_file_.open(filename_, std::ios::out );

  if (!csv_file_.is_open()) {
    return false;
  }
  return true;
}

void CSVFileDumpImpl::CloseCSVFile() {
  if (csv_file_.is_open())
    csv_file_.close();
}

void CSVFileDumpImpl::setFileName(std::string fileName) {
    filename_ = fileName;    
}

// Overload
void DataType2CSVDump::InitCSVFileHeader(){
#if 0
    csv_file_ << "//time,x,y,z,Inten-sity, return#, noise confidence intensity, noise confidence spatial \n";
#endif
    csv_file_ << "x,y,z,intensity,noise\n";
}

// Overload
void DataType2CSVDump::SaveEthPacketToCSVFile(std::list<LvxBasePackDetail> &point_packet_list_temp){
  auto iter = point_packet_list_temp.begin();
  LivoxExtendRawPoint* pointStruct = nullptr;
  int zeroORtwo = 0;

  float x,y,z;
  uint8_t intensity,noise;
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  int end_line = 0;

  //int idx = 0;
  // Iterate every packet
  for (; iter != point_packet_list_temp.end(); iter++) {
    // Iterate every point (96,SINGLE_POINT_NUM)
    pointStruct = (LivoxExtendRawPoint*)&(iter->raw_point);
    //printf("[SaveEthPacketToCSVFile] %d \n", idx++);
    for(int index=0;index < SINGLE_POINT_NUM;index++){

        // Timestamp
        //zeroORtwo = index % 2;
        //timestamp_sec += float(not(zeroORtwo)) * 0.00001;

        // X coordinate pointStruct[index]
        
        //csv_file_ << std::fixed << std::setprecision(3) << pointStruct[index].x/1000;
        x = {(float)pointStruct[index].x/(float)1000};

        //csv_file_ << ",";

        // Y coordinate
        //csv_file_ << std::fixed << std::setprecision(3) << pointStruct[index].y/1000;
        y = {(float)pointStruct[index].y/(float)1000};

        //csv_file_ << ",";

        // Z coordinate
        //csv_file_ << std::fixed << std::setprecision(3) << pointStruct[index].z/1000;
        z = {(float)pointStruct[index].z/(float)1000};

        //csv_file_ << ",";

        // intensity
        //csv_file_ << pointStruct[index].reflectivity;
        intensity = pointStruct[index].reflectivity;

        //csv_file_ << ",";

        // noise
        //csv_file_ << pointStruct[index].tag;
        noise = pointStruct[index].tag;

        //csv_file_ << "\n";
        //csv_file_ << std::endl;

        //print("%.3f,%.3f,%.3f,%d,%d\n",x,y,z,intensity,noise);
        //csv_file_ << std::fixed << std::setprecision(3) << x << "," << y << "," << z << "," << intensity << "," << noise << "\n";
        end_line = snprintf(write_buffer.get(),WRITE_BUFFER_LEN,"%.3f,%.3f,%.3f,%d,%d\n",x,y,z,intensity,noise);
        write_buffer[end_line] = '\0';

        csv_file_ << write_buffer.get();
    }


  }
      
}

LvxFileHandle::LvxFileHandle() : cur_frame_index_(0), cur_offset_(0), frame_duration_(kDefaultFrameDurationTime) {
}

void LvxFileHandle::setFileName(std::string fileName) {
    filename_ = fileName;    
}

bool LvxFileHandle::InitLvxFile() {
  //time_t curtime = time(nullptr);
  //char filename[30] = { 0 };

  //tm* local_time = localtime(&curtime);
  //strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx", local_time);
  //lvx_file_.open(filename, std::ios::out | std::ios::binary);
  lvx_file_.open(filename_, std::ios::out | std::ios::binary);

  if (!lvx_file_.is_open()) {
    return false;
  }
  return true;
}
#if 0
void LvxFileHandle::InitLvxFileHeader() {
  LvxFilePublicHeader lvx_file_public_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = "livox_tech";
  memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());

  lvx_file_public_header.version[0] = 1;
  lvx_file_public_header.version[1] = 1;
  lvx_file_public_header.version[2] = 0;
  lvx_file_public_header.version[3] = 0;

  lvx_file_public_header.magic_code = MAGIC_CODE;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
  cur_offset_ += sizeof(LvxFilePublicHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  LvxFilePrivateHeader lvx_file_private_header = { 0 };
  lvx_file_private_header.frame_duration = frame_duration_;
  lvx_file_private_header.device_count = device_count;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
  cur_offset_ += sizeof(LvxFilePrivateHeader);

  for (int i = 0; i < device_count; i++) {
    memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    cur_offset_ += sizeof(LvxDeviceInfo);
  }

  lvx_file_.write((char *)write_buffer.get(), cur_offset_);
}
#endif
void LvxFileHandle::InitLvxFileHeader() {
  LvxFilePublicHeader lvx_file_public_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = "livox_tech";
  memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());

  lvx_file_public_header.version[0] = 1;
  lvx_file_public_header.version[1] = 1;
  lvx_file_public_header.version[2] = 0;
  lvx_file_public_header.version[3] = 0;

  lvx_file_public_header.magic_code = MAGIC_CODE;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
  cur_offset_ += sizeof(LvxFilePublicHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  LvxFilePrivateHeader lvx_file_private_header = { 0 };
  lvx_file_private_header.frame_duration = frame_duration_;
  lvx_file_private_header.device_count = device_count;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
  cur_offset_ += sizeof(LvxFilePrivateHeader);

  for (int i = 0; i < device_count; i++) {
    memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    cur_offset_ += sizeof(LvxDeviceInfo);
  }

  // [CSV] Change to csv file
#if 0
  lvx_file_.write((char *)write_buffer.get(), cur_offset_);
#endif
}

void LvxFileHandle::SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {
  uint64_t cur_pos = 0;
  FrameHeader frame_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + sizeof(FrameHeader);
  auto iterator = point_packet_list_temp.begin();
  for (; iterator != point_packet_list_temp.end(); iterator++) {
    frame_header.next_offset += iterator->pack_size;
  }

  frame_header.frame_index = cur_frame_index_;

  memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(FrameHeader));
  cur_pos += sizeof(FrameHeader);

  auto iter = point_packet_list_temp.begin();
  for (; iter != point_packet_list_temp.end(); iter++) {
    if (cur_pos + iter->pack_size >= WRITE_BUFFER_LEN) {
      //lvx_file_.write((char*)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
      cur_pos += iter->pack_size;
    }
    else {
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
      cur_pos += iter->pack_size;
    }
  }
  //lvx_file_.write((char*)write_buffer.get(), cur_pos);

  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileHandle::CloseLvxFile() {
  if (lvx_file_.is_open())
    lvx_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet) {
  packet.version = data->version;
  packet.port_id = data->slot;
  packet.lidar_index = data->id;
  packet.rsvd = data->rsvd;
  packet.error_code = data->err_code;
  packet.timestamp_type = data->timestamp_type;
  packet.data_type = data->data_type;
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
  switch (packet.data_type) {
    case PointDataType::kCartesian:
       packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - \
          sizeof(packet.pack_size) + RAW_POINT_NUM*sizeof(LivoxRawPoint);
      memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxRawPoint));
      break;
    case PointDataType::kSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +RAW_POINT_NUM*sizeof(LivoxSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxSpherPoint));
      break;
    case PointDataType::kExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +SINGLE_POINT_NUM*sizeof(LivoxExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, SINGLE_POINT_NUM*sizeof(LivoxExtendRawPoint));
      break;
    case PointDataType::kExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +SINGLE_POINT_NUM*sizeof(LivoxExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, SINGLE_POINT_NUM*sizeof(LivoxExtendSpherPoint));
      break;
    case PointDataType::kDualExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +DUAL_POINT_NUM*sizeof(LivoxDualExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, DUAL_POINT_NUM*sizeof(LivoxDualExtendRawPoint));
      break;
    case PointDataType::kDualExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +DUAL_POINT_NUM*sizeof(LivoxDualExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, DUAL_POINT_NUM*sizeof(LivoxDualExtendSpherPoint));
      break;
    case PointDataType::kImu :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +IMU_POINT_NUM*sizeof(LivoxImuPoint);
      memcpy(packet.raw_point,(void *)data->data, IMU_POINT_NUM*sizeof(LivoxImuPoint));
      break;
    case PointDataType::kTripleExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendRawPoint));
      break;
    case PointDataType::kTripleExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendSpherPoint));
      break;
    default:
      break;
  }
}

void ParseExtrinsicXml(DeviceItem &item, LvxDeviceInfo &info) {
  rapidxml::file<> extrinsic_param("extrinsic.xml");
  rapidxml::xml_document<> doc;
  doc.parse<0>(extrinsic_param.data());
  rapidxml::xml_node<>* root = doc.first_node();
  if ("Livox" == (std::string)root->name()) {
    for (rapidxml::xml_node<>* device = root->first_node(); device; device = device->next_sibling()) {
      if ("Device" == (std::string)device->name() && (strncmp(item.info.broadcast_code, device->value(), kBroadcastCodeSize) == 0)) {
        memcpy(info.lidar_broadcast_code, device->value(), kBroadcastCodeSize);
        memset(info.hub_broadcast_code, 0, kBroadcastCodeSize);
        info.device_type = item.info.type;
        info.device_index = item.handle;
        for (rapidxml::xml_attribute<>* param = device->first_attribute(); param; param = param->next_attribute()) {
          if ("roll" == (std::string)param->name()) info.roll = static_cast<float>(atof(param->value()));
          if ("pitch" == (std::string)param->name()) info.pitch = static_cast<float>(atof(param->value()));
          if ("yaw" == (std::string)param->name()) info.yaw = static_cast<float>(atof(param->value()));
          if ("x" == (std::string)param->name()) info.x = static_cast<float>(atof(param->value()));
          if ("y" == (std::string)param->name()) info.y = static_cast<float>(atof(param->value()));
          if ("z" == (std::string)param->name()) info.z = static_cast<float>(atof(param->value()));
        }
      }
    }
  }
}
