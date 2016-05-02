#include <map>
#include <string>
#include <vector>

#include <ros/init.h>
#include <ros/param.h>

#include <utility_headers/param.hpp>

#include <boost/array.hpp>

#include <gtest/gtest.h>

namespace uhp = utility_headers::param;

TEST(TestParam, CaseBool) {
    // When no parameter is available for the given name on the parameter server,
    // uhp::get should fail and uhp::param should return the given default value
    uhp::del("~bool_param");
    {
        bool val;
        EXPECT_FALSE(uhp::get("~bool_param", val));
    }
    EXPECT_TRUE(uhp::param("~bool_param", true));
    EXPECT_FALSE(uhp::param("~bool_param", false));

    // When a parameter is available on the parameter server,
    // uhp::get should succeed and uhp::param should return the parameter
    uhp::set("~bool_param", true);
    {
        bool val(false);
        EXPECT_TRUE(uhp::get("~bool_param", val));
        EXPECT_TRUE(val);
    }
    uhp::set("~bool_param", false);
    EXPECT_FALSE(uhp::param("~bool_param", true));
}

TEST(TestParam, CaseArithmetic) {
    // When no parameter is available on the parameter server ...
    uhp::del("~arithmetic_param");
    {
        int val;
        EXPECT_FALSE(uhp::get("~arithmetic_param", val));
    }
    EXPECT_EQ(3.14, uhp::param("~arithmetic_param", 3.14));

    // When a parameter is available on the parameter server ...
    uhp::set("~arithmetic_param", 3.14);
    // should not match because the parameter will be loaded as an iteger
    EXPECT_NE(3.14, uhp::param("~arithmetic_param", 10));
    // followings should match the parameter will be loaded as a floating point
    EXPECT_EQ(3.14, uhp::param<double>("~arithmetic_param", 10));
    EXPECT_EQ(3.14, uhp::param("~arithmetic_param", 10.));
    {
        double val(0.);
        EXPECT_TRUE(uhp::get("~arithmetic_param", val));
        EXPECT_EQ(3.14, val);
    }
}

TEST(TestParam, CaseVector) {
    // When no parameter is available on the parameter server ...
    uhp::del("~vector_param");
    {
        std::vector<int> val;
        EXPECT_FALSE(uhp::get("~vector_param", val));
    }
    EXPECT_EQ(10, uhp::param("~vector_param", std::vector<int>(10, 1)).size());

    // When a parameter is available on the parameter server ...
    uhp::set("~vector_param", std::vector<int>(0));
    EXPECT_TRUE(uhp::param("~vector_param", std::vector<int>(10)).empty());
    uhp::set("~vector_param", std::vector<int>(10, -1));
    {
        // should be able to be loaded as a vector of another arithmetic type
        std::vector<double> val;
        EXPECT_TRUE(uhp::get("~vector_param", val));
        EXPECT_EQ(10, val.size());
    }
}

TEST(TestParam, CaseArray) {
    // When no parameter is available on the parameter server ...
    uhp::del("~array_param");
    EXPECT_EQ(3, uhp::param("~array_param", boost::array<int, 3>()).size());
    EXPECT_EQ(10, uhp::param("~array_param", boost::array<int, 10>()).size());

    // When a parameter is available on the parameter server ...
    {
        const boost::array<double, 3> val = {1.1, 2.2, 3.3};
        uhp::set("~array_param", val);
    }
    {
        boost::array<double, 3> val;
        EXPECT_TRUE(uhp::get("~array_param", val));
        EXPECT_TRUE(val[0] == 1.1 && val[1] == 2.2 && val[2] == 3.3);
    }
    {
        // uhp::get should fail if the array size is wrong
        boost::array<double, 10> val;
        EXPECT_FALSE(uhp::get("~array_param", val));
    }
    {
        // should be able to be loaded another array type of another arithmetic type
        const std::vector<int> val(uhp::param("~array_param", std::vector<int>()));
        EXPECT_TRUE(val[0] == 1 && val[1] == 2 && val[2] == 3);
    }
}

TEST(TestParam, CaseMap) {
    // When no parameter is available on the parameter server ...
    uhp::del("~map_param");
    {
        std::map<std::string, double> default_val;
        EXPECT_TRUE(uhp::param("~map_param", default_val).empty());
        default_val["pi"] = 3.14;
        default_val["e"] = 2.78;
        EXPECT_EQ(2, uhp::param("~map_param", default_val).size());
    }

    // When a parameter is available on the parameter server ...
    {
        std::map<std::string, std::string> val;
        val["foo"] = "bar";
        val["ping"] = "pong";
        uhp::set("~map_param", val);
    }
    {
        const std::map<std::string, std::string> val(
            uhp::param("~map_param", std::map<std::string, std::string>()));
        EXPECT_EQ(2, val.size());
        EXPECT_TRUE(val.count("foo") == 1 && val.count("ping") == 1);
    }
}

TEST(TestParam, CaseComplex) {
    // Dislike ros::param, nested arrays, vectors, and maps should be acceptable
    {
        std::map<std::string, std::vector<unsigned int> > src;
        src["even"].push_back(0);
        src["even"].push_back(2);
        src["even"].push_back(4);
        src["prime"].push_back(2);
        src["prime"].push_back(3);
        src["prime"].push_back(5);
        uhp::set("~vector_map_param", src);
    }
    {
        std::map<std::string, std::vector<int> > val(
            uhp::param("~vector_map_param", std::map<std::string, std::vector<int> >()));
        EXPECT_TRUE(val.count("even") == 1 && val.count("prime") == 1);
        EXPECT_TRUE(val["even"].size() == 3 && val["prime"].size() == 3);
    }
    {
        std::map<std::string, boost::array<double, 3> > val;
        EXPECT_TRUE(uhp::get("~vector_map_param", val));
        EXPECT_TRUE(val.count("even") == 1 && val.count("prime") == 1);
        EXPECT_TRUE(val["even"][0] == 0 && val["even"][1] == 2 && val["even"][2] == 4);
        EXPECT_TRUE(val["prime"][0] == 2 && val["prime"][1] == 3 && val["prime"][2] == 5);
    }

    typedef boost::array<std::map<std::string, boost::array<std::map<std::string, double>, 5> >, 3>
        ManyNestedType;
    uhp::del("~many_nested_param");
    {
        ManyNestedType val;
        EXPECT_FALSE(uhp::get("~many_nested_param", val));
        val[0]["alpha"][0]["asahi"] = 1.;
        val[0]["bravo"][3]["iroha"] = 2.;
        val[2]["charlie"][4]["ueno"] = 3.;
        uhp::set("~many_nested_param", val);
    }
    {
        ManyNestedType val;
        EXPECT_TRUE(uhp::get("~many_nested_param", val));
        EXPECT_EQ(1., val[0]["alpha"][0]["asahi"]);
        EXPECT_EQ(2., val[0]["bravo"][3]["iroha"]);
        EXPECT_EQ(3., val[2]["charlie"][4]["ueno"]);
    }
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_param");

    return RUN_ALL_TESTS();
}
