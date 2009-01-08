/*
 * hokuyo_urg.h
 *
 *  Created on: Dec 25, 2008
 *      Author: mahisorn
 */

#ifndef HOKUYO_URG_H_
#define HOKUYO_URG_H_

#include "serial_file.h"

#define URG_4LX_NUM_STEP            769
#define URG_4LX_START_DEG           (-135)
#define URG_4LX_START_RAD           ((URG_4LX_START_DEG * M_PI)/180.0)
#define URG_4LX_DEG_STEP            (270.0/768)
#define URG_4LX_RAD_STEP            ((URG_4LX_DEG_STEP * M_PI)/180.0)
#define URG_4LX_START_STEP          44
#define URG_4LX_END_STEP            625

#define URG_ENABLE_SCIP20           "SCIP2.0\n"

#define URG_SCIP10_NO_ERROR         "0\n"
#define URG_SCIP10_GET_VERSION      "V\n"
#define URG_SCIP10_LASER_ON         "L0\n"
#define URG_SCIP10_LASER_OFF        "L1\n"

#define URG_SCIP20_NO_ERROR         "00P\n"
#define URG_SCIP20_GET_VERSION      "VV\n"
#define URG_SCIP20_LASER_ON         "BM\n"
#define URG_SCIP20_LASER_OFF        "QT\n"

#define hokuyo_urg_check_open() \
    if(!is_open()) \
        throw librobotics::LibRoboticsRuntimeException( \
                "Device is not open in %s", __FUNCTION__)

#define hokuyo_urg_get_data() \
    if(fgets(buff, sizeof(buff), this->file) == NULL) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Get data error in %s", __FUNCTION__); \
    }

#define hokuyo_urg_get_data2() \
    if(fgets(buff, sizeof(buff), this->file) == NULL) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Get data error in %s", __FUNCTION__); \
    }\
    if(!check_sum_scip2()) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Data check sum error in %s", __FUNCTION__); \
    }

#define hokuyo_urg_cmd_cmp(cmd) \
    hokuyo_urg_get_data()\
    if (strcmp(cmd, buff) != 0) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Command error in %s", __FUNCTION__); \
    }

#define hokuyo_urg_status_cmp(status) \
    hokuyo_urg_get_data()\
    if (strcmp(status, buff) != 0) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Status error in %s", __FUNCTION__); \
    }

#define hokuyo_urg_status_cmp2(status) \
    hokuyo_urg_get_data2()\
    if (strcmp(status, buff) != 0) { \
        throw librobotics::LibRoboticsRuntimeException( \
                "Status error in %s", __FUNCTION__); \
    }


class hokuyo_urg : public serial_file {
public:


    hokuyo_urg() : serial_file()
    { }

    ~hokuyo_urg() { }

    bool enable_scip20() {
        hokuyo_urg_check_open();

        fprintf(this->file, URG_ENABLE_SCIP20);

        //echo command
        hokuyo_urg_cmd_cmp(URG_ENABLE_SCIP20);

        //check status
        hokuyo_urg_status_cmp(URG_SCIP10_NO_ERROR);

        //extra LF
        hokuyo_urg_get_data();
        return true;
    }



    std::string get_version_scip10() {
        hokuyo_urg_check_open();
        std::string tmp;

        //send command
        fprintf(this->file, URG_SCIP10_GET_VERSION);


        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP10_GET_VERSION);

        //error status
        hokuyo_urg_status_cmp(URG_SCIP10_NO_ERROR);

        //Vender Information
        hokuyo_urg_get_data();
        tmp += buff;

        //Product Information
        hokuyo_urg_get_data();
        tmp += buff;

        //Firmware Version
        hokuyo_urg_get_data();
        tmp += buff;

        //Protocol Version
        hokuyo_urg_get_data();
        tmp += buff;

        //Sensor Serial Number
        hokuyo_urg_get_data();
        tmp += buff;

        //Sensor status
        hokuyo_urg_get_data();
        tmp += buff;

        //extra LF
        hokuyo_urg_get_data();

        return tmp;
    }

    std::string get_sn_scip10() {
        std::string tmp = get_version_scip10();
        int idx = tmp.find("SERI:");
        return tmp.substr (idx + 5, 8);
    }

    void laser_on_scip10() {
        hokuyo_urg_check_open();

        fprintf(this->file, URG_SCIP10_LASER_ON);

        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP10_LASER_ON);

        //error status
        hokuyo_urg_status_cmp(URG_SCIP10_NO_ERROR);

        hokuyo_urg_get_data();
    }

    void laser_off_scip10() {
        hokuyo_urg_check_open();

        fprintf(this->file, URG_SCIP10_LASER_OFF);

        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP10_LASER_OFF);

        //error status
        hokuyo_urg_status_cmp(URG_SCIP10_NO_ERROR);

        //last LF
        hokuyo_urg_get_data();
    }

//    void set_baudrate_scip10(int baudrate) {
//        librobotics::LibRoboticsRuntimeException("Not implement in %s", __FUNCTION__);
//    }

    template<typename  T>
    void get_data_scip10(int begin, int end,  int cluster, std::vector<T>& ranges) {
        hokuyo_urg_check_open();

        //send command
        char cmd[16];
        sprintf(cmd, "G%03d%03d%02d\n", begin, end, cluster);
        fprintf(this->file, "%s", cmd);

        //echo command
        hokuyo_urg_cmd_cmp(cmd);

        //error status
        hokuyo_urg_status_cmp(URG_SCIP10_NO_ERROR);

        int i = 0;
        int j = 0;
        size_t size = ((end - begin) + 1);
        if(cluster > 1) {
            size = (int)(ceil((float)size / cluster));
        }

        if(ranges.size() != size)
            ranges.resize(size);

        while (1) {
            hokuyo_urg_get_data();

            if (!strcmp("\n", buff)) {
               //end of result
               break;
            }

            for (i = 0; i < (int)strlen(buff) - 1; ++i , ++j) {
               // Note: It is assumed that received data is completed
               // (Problem will occour if data is incomplete)
               ranges[j] = (T)((unsigned int)(((buff[i] - 0x30) << 6) + buff[i+1] - 0x30));
               i++;
            }
        }
    }

    bool check_sum_scip2() {
        if(strlen(buff) < 2) return false;

        unsigned int sum = 0;
        char chk_sum = 0;
        for(size_t i = 0; i < (strlen(buff) - 2); i++) {
            sum += (unsigned int)buff[i];
        }
        chk_sum = ((sum & 0x3f) + 0x30);
        return (buff[strlen(buff) - 2] == chk_sum);
    }

    std::string get_version_scip20() {
        hokuyo_urg_check_open();

        std::string tmp;

        fprintf(this->file, URG_SCIP20_GET_VERSION);

        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP20_GET_VERSION);

        //error status
        hokuyo_urg_status_cmp2(URG_SCIP20_NO_ERROR);

        //Vender Information
        hokuyo_urg_get_data();
        tmp += buff;

        //Product Information
        hokuyo_urg_get_data();
        tmp += buff;

        //Firmware Version
        hokuyo_urg_get_data();
        tmp += buff;

        //Protocol Version
        hokuyo_urg_get_data();
        tmp += buff;

        //Sensor Serial Number
        hokuyo_urg_get_data();
        tmp += buff;

        //extra LF
        hokuyo_urg_get_data();

        return tmp;
    }

    std::string get_sn_scip20() {
        std::string tmp = get_version_scip20();
        int idx = tmp.find("SERI:");
        return tmp.substr (idx + 5, 8);
    }

    void laser_on_scip20() {
        hokuyo_urg_check_open();

        fprintf(this->file, URG_SCIP20_LASER_ON);

        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP20_LASER_ON);

        //error status
        hokuyo_urg_get_data2();
        if(!(strcmp(buff, URG_SCIP20_NO_ERROR) == 0 || strcmp(buff, "02R\n") == 0)) {
            throw librobotics::LibRoboticsRuntimeException(
                    "Cannot turn on laser in %s", __FUNCTION__);
        }

        hokuyo_urg_get_data();
    }

    void laser_off_scip20() {
        hokuyo_urg_check_open();

        fprintf(this->file, URG_SCIP20_LASER_OFF);

        //echo command
        hokuyo_urg_cmd_cmp(URG_SCIP20_LASER_OFF);

        //error status
        hokuyo_urg_status_cmp2(URG_SCIP20_NO_ERROR);

        //last LF
        hokuyo_urg_get_data();
    }

    template<typename  T>
    void get_data_2char_scip20(int begin, int end,  int cluster,
                               std::vector<T>& ranges,
                               unsigned int& time)
    {
        //send command
        char cmd[16];
        sprintf(cmd, "GS%04d%04d%02d\n", begin, end, cluster);
        fprintf(this->file, "%s", cmd);

        //echo command
        hokuyo_urg_cmd_cmp(cmd);

        //error status
        hokuyo_urg_status_cmp2(URG_SCIP20_NO_ERROR);

        //time stamp
        hokuyo_urg_get_data2();
        time = (unsigned int)( ((buff[0] - 0x30) << 18 ) +
                               ((buff[1] - 0x30) << 12 ) +
                               ((buff[2] - 0x30) << 6  ) +
                                (buff[3] - 0x30) );

        int i = 0;
        int j = 0;
        size_t size = ((end - begin) + 1);
        if(cluster != 1) {
            size = (int)(ceil((float)size / cluster));
        }
        if(ranges.size() != size)
           ranges.resize(size);

        while (1) {
            hokuyo_urg_get_data();
            if (strcmp("\n", buff) == 0) {
              //end of result
                break;
            }

            if(!check_sum_scip2()) {
                throw librobotics::LibRoboticsRuntimeException( \
                        "Data check sum error in %s", __FUNCTION__);
            }

            for (i = 0; i < (int)strlen(buff) - 2; ++i , ++j) {
              // Note: It is assumed that received data is completed
              // (Problem will occour if data is incomplete)
              ranges[j] = (T)((unsigned int)(((buff[i] - 0x30) << 6) + buff[i+1] - 0x30));
              i++;
            }
        }
    }

    template<typename  T>
    void get_data_3char_scip20(int begin, int end,  int cluster,
                               std::vector<T>& ranges,
                               unsigned int& time)
    {
        //send command
        char cmd[16];
        sprintf(cmd, "GD%04d%04d%02d\n", begin, end, cluster);
        fprintf(this->file, "%s", cmd);

        //echo command
        hokuyo_urg_cmd_cmp(cmd);

        //error status
        hokuyo_urg_status_cmp2(URG_SCIP20_NO_ERROR);

        //time stamp
        hokuyo_urg_get_data2();
        time = (unsigned int)( ((buff[0] - 0x30) << 18 ) +
                               ((buff[1] - 0x30) << 12 ) +
                               ((buff[2] - 0x30) << 6  ) +
                               ((buff[3] - 0x30)       ) );

        size_t i = 0;
        size_t j = 0;
        size_t size = ((end - begin) + 1);
        if(cluster > 1) {
            size = (int)(ceil((float)size / cluster));
        }
        if(ranges.size() != size)
           ranges.resize(size);

        std::string tmp;
        while (1) {
            hokuyo_urg_get_data();

            if (strcmp("\n", buff) == 0) {
              //end of result
                break;
            }

            if(!check_sum_scip2()) {
                throw librobotics::LibRoboticsRuntimeException( \
                        "Data check sum error in %s", __FUNCTION__);
            }

            buff[strlen(buff) - 2] = '\0';
            tmp += buff;
        }
        const char * c_str = tmp.c_str();
        for (i = 0; i < tmp.length() - 2; i+=3 , ++j) {
            // Note: It is assumed that received data is completed
            // (Problem will occur if data is incomplete)
            ranges[j] = (T)( (unsigned int)( ((c_str[i]   - 0x30) << 12) +
                                             ((c_str[i+1] - 0x30) << 6 ) +
                                             ((c_str[i+2] - 0x30)      ) ));
        }
    }

protected:
    char buff[80];

};


#endif /* HOKUYO_URG_H_ */
