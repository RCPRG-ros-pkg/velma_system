#include "velma_gazebo.h"

    bool VelmaGazebo::parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c)
    {
        const char* link1_str = c->Attribute("link1");
        const char* link2_str = c->Attribute("link2");
        if (link1_str != NULL && link2_str != NULL)
        {
            link1 = link1_str;
            link2 = link2_str;
            return true;
        }
        std::cout<< "disable_collisions has wrong attributes" << std::endl;

        return false;
    }

    bool VelmaGazebo::parseSRDF(const std::string &xml_string, std::vector<std::pair<std::string, std::string> > &disabled_collisions)
    {
        disabled_collisions.clear();

        TiXmlDocument xml_doc;
        xml_doc.Parse(xml_string.c_str());
        if (xml_doc.Error())
        {
//            ROS_ERROR("%s", xml_doc.ErrorDesc());
            xml_doc.ClearError();
            return false;
        }

        TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
        if (!robot_xml)
        {
//            ROS_ERROR("Could not find the 'robot' element in the xml file");
            return false;
        }
        // Get all disable_collisions elements
        for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions"); disable_collision_xml; disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
        {
            std::string link1, link2;
            try {
                if (!parseDisableCollision(link1, link2, disable_collision_xml)) {
                    return false;
                }
                disabled_collisions.push_back(std::make_pair(link1, link2));
            }
            catch (urdf::ParseError &e) {
//                ROS_ERROR("disable_collisions xml is not initialized correctly");
                return false;
            }
        }
        return true;
    }

