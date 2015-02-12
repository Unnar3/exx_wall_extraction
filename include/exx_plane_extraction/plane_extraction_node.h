#ifndef __PLANE_EXTRACTION_NODE_H
#define __PLANE_EXTRACTION_NODE_H

#include <string>

namespace exx {
    namespace plane_extraction_node {
        const std::string NODE_NAME =                 "plane_extraction";
        const std::string TOPIC_PWM =                 "/arduino/pwm";
        const std::string TOPIC_ENCODERS =            "/arduino/encoders";
        const std::string TOPIC_TWIST =               "/s8/twist";
        const std::string TOPIC_ACTUAL_TWIST =        "/s8/actual_twist";
    }
}

#endif