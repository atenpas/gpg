/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef CONFIG_FILE_H_
#define CONFIG_FILE_H_


#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <typeinfo>


class ConfigFile
{
  public:

    ConfigFile(const std::string &fName);

    bool keyExists(const std::string &key) const;

    template <typename ValueType>
    ValueType getValueOfKey(const std::string &key, ValueType const &defaultValue) const
    {
      if (!keyExists(key))
        return defaultValue;

      return string_to_T<ValueType>(contents.find(key)->second);
    }

    std::string getValueOfKeyAsString(const std::string &key, const std::string &defaultValue);
    
    template <typename T>
    std::string T_to_string(T const &val) const
    {
      std::ostringstream ostr;
      ostr << val;

      return ostr.str();
    }
    
    template <typename T>
    T string_to_T(std::string const &val) const
    {
      std::istringstream istr(val);
      T returnVal;
      if (!(istr >> returnVal))
        std::cout << "CFG: Not a valid " + (std::string)typeid(T).name() + " received!\n";

      return returnVal;
    }


  private:

    void removeComment(std::string &line) const;

    bool onlyWhitespace(const std::string &line) const;

    bool validLine(const std::string &line) const;

    void extractKey(std::string &key, size_t const &sepPos, const std::string &line) const;

    void extractValue(std::string &value, size_t const &sepPos, const std::string &line) const;

    void extractContents(const std::string &line);

    void parseLine(const std::string &line, size_t const lineNo);

    void ExtractKeys();

    std::map<std::string, std::string> contents;
    std::string fName;
};

#endif /* CONFIG_FILE_H_ */
