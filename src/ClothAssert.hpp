// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#ifndef CLOTHASSERT_HPP
#define CLOTHASSERT_HPP 1

#include <cstdlib> 
#include <string>
#include <stdexcept>

namespace cloth
{

static inline void AssertHandler(bool cond, const std::string& file, const int& line)
{
	if (!cond)
	{
		std::string err_msg = "Assertion failed in "+file+" line "+std::to_string(line);
		throw std::runtime_error(err_msg.c_str());
	}
}

static inline void AssertHandlerMsg(bool cond, const std::string& file, const int& line, const std::string &msg)
{
	if (!cond)
	{
		std::string err_msg = "Assertion failed in "+file+" line "+std::to_string(line)+": "+msg;
		throw std::runtime_error(err_msg.c_str());
	}
}

}

// Neat trick that allows macros with multiple arguments:
// https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros
// Also consider using #condition

#define ClothAssert_withmsg(cond, msg) AssertHandlerMsg(cond, std::string(__FILE__), __LINE__, msg)
#define ClothAssert_nomsg(cond) AssertHandler(cond, std::string(__FILE__), __LINE__)
#define ClothAssert_stripargs(xx,cond,msg,FUNC, ...) FUNC

#define ClothAssert(...) \
	ClothAssert_stripargs(,##__VA_ARGS__,\
	ClothAssert_withmsg(__VA_ARGS__),\
	ClothAssert_nomsg(__VA_ARGS__),\
	)

#endif
