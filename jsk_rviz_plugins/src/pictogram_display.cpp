// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/

#include "pictogram_display.h"
#include <QPainter>
#include <QFontDatabase>
#include <ros/package.h>

////////////////////////////////////////////////////////
// read Entypo fonts
// http://mempko.wordpress.com/2014/11/28/using-entypo-fonts-as-icons-in-your-qt-application/
////////////////////////////////////////////////////////

#include "Entypo.dat"
#include "Entypo_Social.dat"

namespace jsk_rviz_plugin
{

  void PictogramObject::setupCharacterMap()
  {
    entypo_character_map_["phone"] = QString::fromWCharArray(L"\x1F4DE");
    entypo_character_map_["mobile"] = QString::fromWCharArray(L"\x1F4F1");
    entypo_character_map_["mouse"] = QString::fromWCharArray(L"\xE789");
    entypo_character_map_["address"] = QString::fromWCharArray(L"\xE723");
    entypo_character_map_["mail"] = QString::fromWCharArray(L"\x2709");
    entypo_character_map_["paper-plane"] = QString::fromWCharArray(L"\x1F53F");
    entypo_character_map_["pencil"] = QString::fromWCharArray(L"\x270E");
    entypo_character_map_["feather"] = QString::fromWCharArray(L"\x2712");
    entypo_character_map_["attach"] = QString::fromWCharArray(L"\x1F4CE");
    entypo_character_map_["inbox"] = QString::fromWCharArray(L"\xE777");
    entypo_character_map_["reply"] = QString::fromWCharArray(L"\xE712");
    entypo_character_map_["reply-all"] = QString::fromWCharArray(L"\xE713");
    entypo_character_map_["forward"] = QString::fromWCharArray(L"\x27A6");
    entypo_character_map_["user"] = QString::fromWCharArray(L"\x1F464");
    entypo_character_map_["users"] = QString::fromWCharArray(L"\x1F465");
    entypo_character_map_["add-user"] = QString::fromWCharArray(L"\xE700");
    entypo_character_map_["vcard"] = QString::fromWCharArray(L"\xE722");
    entypo_character_map_["export"] = QString::fromWCharArray(L"\xE715");
    entypo_character_map_["location"] = QString::fromWCharArray(L"\xE724");
    entypo_character_map_["map"] = QString::fromWCharArray(L"\xE727");
    entypo_character_map_["compass"] = QString::fromWCharArray(L"\xE728");
    entypo_character_map_["direction"] = QString::fromWCharArray(L"\x27A2");
    entypo_character_map_["hair-cross"] = QString::fromWCharArray(L"\x1F3AF");
    entypo_character_map_["share"] = QString::fromWCharArray(L"\xE73C");
    entypo_character_map_["shareable"] = QString::fromWCharArray(L"\xE73E");
    entypo_character_map_["heart"] = QString::fromWCharArray(L"\x2665");
    entypo_character_map_["heart-empty"] = QString::fromWCharArray(L"\x2661");
    entypo_character_map_["star"] = QString::fromWCharArray(L"\x2605");
    entypo_character_map_["star-empty"] = QString::fromWCharArray(L"\x2606");
    entypo_character_map_["thumbs-up"] = QString::fromWCharArray(L"\x1F44D");
    entypo_character_map_["thumbs-down"] = QString::fromWCharArray(L"\x1F44E");
    entypo_character_map_["chat"] = QString::fromWCharArray(L"\xE720");
    entypo_character_map_["comment"] = QString::fromWCharArray(L"\xE718");
    entypo_character_map_["quote"] = QString::fromWCharArray(L"\x275E");
    entypo_character_map_["home"] = QString::fromWCharArray(L"\x2302");
    entypo_character_map_["popup"] = QString::fromWCharArray(L"\xE74C");
    entypo_character_map_["search"] = QString::fromWCharArray(L"\x1F50D");
    entypo_character_map_["flashlight"] = QString::fromWCharArray(L"\x1F526");
    entypo_character_map_["print"] = QString::fromWCharArray(L"\xE716");
    entypo_character_map_["bell"] = QString::fromWCharArray(L"\x1F514");
    entypo_character_map_["link"] = QString::fromWCharArray(L"\x1F517");
    entypo_character_map_["flag"] = QString::fromWCharArray(L"\x2691");
    entypo_character_map_["cog"] = QString::fromWCharArray(L"\x2699");
    entypo_character_map_["tools"] = QString::fromWCharArray(L"\x2692");
    entypo_character_map_["trophy"] = QString::fromWCharArray(L"\x1F3C6");
    entypo_character_map_["tag"] = QString::fromWCharArray(L"\xE70C");
    entypo_character_map_["camera"] = QString::fromWCharArray(L"\x1F4F7");
    entypo_character_map_["megaphone"] = QString::fromWCharArray(L"\x1F4E3");
    entypo_character_map_["moon"] = QString::fromWCharArray(L"\x263D");
    entypo_character_map_["palette"] = QString::fromWCharArray(L"\x1F3A8");
    entypo_character_map_["leaf"] = QString::fromWCharArray(L"\x1F342");
    entypo_character_map_["note"] = QString::fromWCharArray(L"\x266A");
    entypo_character_map_["beamed-note"] = QString::fromWCharArray(L"\x266B");
    entypo_character_map_["new"] = QString::fromWCharArray(L"\x1F4A5");
    entypo_character_map_["graduation-cap"] = QString::fromWCharArray(L"\x1F393");
    entypo_character_map_["book"] = QString::fromWCharArray(L"\x1F4D5");
    entypo_character_map_["newspaper"] = QString::fromWCharArray(L"\x1F4F0");
    entypo_character_map_["bag"] = QString::fromWCharArray(L"\x1F45C");
    entypo_character_map_["airplane"] = QString::fromWCharArray(L"\x2708");
    entypo_character_map_["lifebuoy"] = QString::fromWCharArray(L"\xE788");
    entypo_character_map_["eye"] = QString::fromWCharArray(L"\xE70A");
    entypo_character_map_["clock"] = QString::fromWCharArray(L"\x1F554");
    entypo_character_map_["mic"] = QString::fromWCharArray(L"\x1F3A4");
    entypo_character_map_["calendar"] = QString::fromWCharArray(L"\x1F4C5");
    entypo_character_map_["flash"] = QString::fromWCharArray(L"\x26A1");
    entypo_character_map_["thunder-cloud"] = QString::fromWCharArray(L"\x26C8");
    entypo_character_map_["droplet"] = QString::fromWCharArray(L"\x1F4A7");
    entypo_character_map_["cd"] = QString::fromWCharArray(L"\x1F4BF");
    entypo_character_map_["briefcase"] = QString::fromWCharArray(L"\x1F4BC");
    entypo_character_map_["air"] = QString::fromWCharArray(L"\x1F4A8");
    entypo_character_map_["hourglass"] = QString::fromWCharArray(L"\x23F3");
    entypo_character_map_["gauge"] = QString::fromWCharArray(L"\x1F6C7");
    entypo_character_map_["language"] = QString::fromWCharArray(L"\x1F394");
    entypo_character_map_["network"] = QString::fromWCharArray(L"\xE776");
    entypo_character_map_["key"] = QString::fromWCharArray(L"\x1F511");
    entypo_character_map_["battery"] = QString::fromWCharArray(L"\x1F50B");
    entypo_character_map_["bucket"] = QString::fromWCharArray(L"\x1F4FE");
    entypo_character_map_["magnet"] = QString::fromWCharArray(L"\xE7A1");
    entypo_character_map_["drive"] = QString::fromWCharArray(L"\x1F4FD");
    entypo_character_map_["cup"] = QString::fromWCharArray(L"\x2615");
    entypo_character_map_["rocket"] = QString::fromWCharArray(L"\x1F680");
    entypo_character_map_["brush"] = QString::fromWCharArray(L"\xE79A");
    entypo_character_map_["suitcase"] = QString::fromWCharArray(L"\x1F6C6");
    entypo_character_map_["traffic-cone"] = QString::fromWCharArray(L"\x1F6C8");
    entypo_character_map_["globe"] = QString::fromWCharArray(L"\x1F30E");
    entypo_character_map_["keyboard"] = QString::fromWCharArray(L"\x2328");
    entypo_character_map_["browser"] = QString::fromWCharArray(L"\xE74E");
    entypo_character_map_["publish"] = QString::fromWCharArray(L"\xE74D");
    entypo_character_map_["progress-3"] = QString::fromWCharArray(L"\xE76B");
    entypo_character_map_["progress-2"] = QString::fromWCharArray(L"\xE76A");
    entypo_character_map_["progress-1"] = QString::fromWCharArray(L"\xE769");
    entypo_character_map_["progress-0"] = QString::fromWCharArray(L"\xE768");
    entypo_character_map_["light-down"] = QString::fromWCharArray(L"\x1F505");
    entypo_character_map_["light-up"] = QString::fromWCharArray(L"\x1F506");
    entypo_character_map_["adjust"] = QString::fromWCharArray(L"\x25D1");
    entypo_character_map_["code"] = QString::fromWCharArray(L"\xE714");
    entypo_character_map_["monitor"] = QString::fromWCharArray(L"\x1F4BB");
    entypo_character_map_["infinity"] = QString::fromWCharArray(L"\x221E");
    entypo_character_map_["light-bulb"] = QString::fromWCharArray(L"\x1F4A1");
    entypo_character_map_["credit-card"] = QString::fromWCharArray(L"\x1F4B3");
    entypo_character_map_["database"] = QString::fromWCharArray(L"\x1F4F8");
    entypo_character_map_["voicemail"] = QString::fromWCharArray(L"\x2707");
    entypo_character_map_["clipboard"] = QString::fromWCharArray(L"\x1F4CB");
    entypo_character_map_["cart"] = QString::fromWCharArray(L"\xE73D");
    entypo_character_map_["box"] = QString::fromWCharArray(L"\x1F4E6");
    entypo_character_map_["ticket"] = QString::fromWCharArray(L"\x1F3AB");
    entypo_character_map_["rss"] = QString::fromWCharArray(L"\xE73A");
    entypo_character_map_["signal"] = QString::fromWCharArray(L"\x1F4F6");
    entypo_character_map_["thermometer"] = QString::fromWCharArray(L"\x1F4FF");
    entypo_character_map_["water"] = QString::fromWCharArray(L"\x1F4A6");
    entypo_character_map_["sweden"] = QString::fromWCharArray(L"\xF601");
    entypo_character_map_["line-graph"] = QString::fromWCharArray(L"\x1F4C8");
    entypo_character_map_["pie-chart"] = QString::fromWCharArray(L"\x25F4");
    entypo_character_map_["bar-graph"] = QString::fromWCharArray(L"\x1F4CA");
    entypo_character_map_["area-graph"] = QString::fromWCharArray(L"\x1F53E");
    entypo_character_map_["lock"] = QString::fromWCharArray(L"\x1F512");
    entypo_character_map_["lock-open"] = QString::fromWCharArray(L"\x1F513");
    entypo_character_map_["logout"] = QString::fromWCharArray(L"\xE741");
    entypo_character_map_["login"] = QString::fromWCharArray(L"\xE740");
    entypo_character_map_["check"] = QString::fromWCharArray(L"\x2713");
    entypo_character_map_["cross"] = QString::fromWCharArray(L"\x274C");
    entypo_character_map_["squared-minus"] = QString::fromWCharArray(L"\x229F");
    entypo_character_map_["squared-plus"] = QString::fromWCharArray(L"\x229E");
    entypo_character_map_["squared-cross"] = QString::fromWCharArray(L"\x274E");
    entypo_character_map_["circled-minus"] = QString::fromWCharArray(L"\x2296");
    entypo_character_map_["circled-plus"] = QString::fromWCharArray(L"\x2295");
    entypo_character_map_["circled-cross"] = QString::fromWCharArray(L"\x2716");
    entypo_character_map_["minus"] = QString::fromWCharArray(L"\x2796");
    entypo_character_map_["plus"] = QString::fromWCharArray(L"\x2795");
    entypo_character_map_["erase"] = QString::fromWCharArray(L"\x232B");
    entypo_character_map_["block"] = QString::fromWCharArray(L"\x1F6AB");
    entypo_character_map_["info"] = QString::fromWCharArray(L"\x2139");
    entypo_character_map_["circled-info"] = QString::fromWCharArray(L"\xE705");
    entypo_character_map_["help"] = QString::fromWCharArray(L"\x2753");
    entypo_character_map_["circled-help"] = QString::fromWCharArray(L"\xE704");
    entypo_character_map_["warning"] = QString::fromWCharArray(L"\x26A0");
    entypo_character_map_["cycle"] = QString::fromWCharArray(L"\x1F504");
    entypo_character_map_["cw"] = QString::fromWCharArray(L"\x27F3");
    entypo_character_map_["ccw"] = QString::fromWCharArray(L"\x27F2");
    entypo_character_map_["shuffle"] = QString::fromWCharArray(L"\x1F500");
    entypo_character_map_["back"] = QString::fromWCharArray(L"\x1F519");
    entypo_character_map_["level-down"] = QString::fromWCharArray(L"\x21B3");
    entypo_character_map_["retweet"] = QString::fromWCharArray(L"\xE717");
    entypo_character_map_["loop"] = QString::fromWCharArray(L"\x1F501");
    entypo_character_map_["back-in-time"] = QString::fromWCharArray(L"\xE771");
    entypo_character_map_["level-up"] = QString::fromWCharArray(L"\x21B0");
    entypo_character_map_["switch"] = QString::fromWCharArray(L"\x21C6");
    entypo_character_map_["numbered-list"] = QString::fromWCharArray(L"\xE005");
    entypo_character_map_["add-to-list"] = QString::fromWCharArray(L"\xE003");
    entypo_character_map_["layout"] = QString::fromWCharArray(L"\x268F");
    entypo_character_map_["list"] = QString::fromWCharArray(L"\x2630");
    entypo_character_map_["text-doc"] = QString::fromWCharArray(L"\x1F4C4");
    entypo_character_map_["text-doc-inverted"] = QString::fromWCharArray(L"\xE731");
    entypo_character_map_["doc"] = QString::fromWCharArray(L"\xE730");
    entypo_character_map_["docs"] = QString::fromWCharArray(L"\xE736");
    entypo_character_map_["landscape-doc"] = QString::fromWCharArray(L"\xE737");
    entypo_character_map_["picture"] = QString::fromWCharArray(L"\x1F304");
    entypo_character_map_["video"] = QString::fromWCharArray(L"\x1F3AC");
    entypo_character_map_["music"] = QString::fromWCharArray(L"\x1F3B5");
    entypo_character_map_["folder"] = QString::fromWCharArray(L"\x1F4C1");
    entypo_character_map_["archive"] = QString::fromWCharArray(L"\xE800");
    entypo_character_map_["trash"] = QString::fromWCharArray(L"\xE729");
    entypo_character_map_["upload"] = QString::fromWCharArray(L"\x1F4E4");
    entypo_character_map_["download"] = QString::fromWCharArray(L"\x1F4E5");
    entypo_character_map_["save"] = QString::fromWCharArray(L"\x1F4BE");
    entypo_character_map_["install"] = QString::fromWCharArray(L"\xE778");
    entypo_character_map_["cloud"] = QString::fromWCharArray(L"\x2601");
    entypo_character_map_["upload-cloud"] = QString::fromWCharArray(L"\xE711");
    entypo_character_map_["bookmark"] = QString::fromWCharArray(L"\x1F516");
    entypo_character_map_["bookmarks"] = QString::fromWCharArray(L"\x1F4D1");
    entypo_character_map_["open-book"] = QString::fromWCharArray(L"\x1F4D6");
    entypo_character_map_["play"] = QString::fromWCharArray(L"\x25B6");
    entypo_character_map_["paus"] = QString::fromWCharArray(L"\x2016");
    entypo_character_map_["record"] = QString::fromWCharArray(L"\x25CF");
    entypo_character_map_["stop"] = QString::fromWCharArray(L"\x25A0");
    entypo_character_map_["ff"] = QString::fromWCharArray(L"\x23E9");
    entypo_character_map_["fb"] = QString::fromWCharArray(L"\x23EA");
    entypo_character_map_["to-start"] = QString::fromWCharArray(L"\x23EE");
    entypo_character_map_["to-end"] = QString::fromWCharArray(L"\x23ED");
    entypo_character_map_["resize-full"] = QString::fromWCharArray(L"\xE744");
    entypo_character_map_["resize-small"] = QString::fromWCharArray(L"\xE746");
    entypo_character_map_["volume"] = QString::fromWCharArray(L"\x23F7");
    entypo_character_map_["sound"] = QString::fromWCharArray(L"\x1F50A");
    entypo_character_map_["mute"] = QString::fromWCharArray(L"\x1F507");
    entypo_character_map_["flow-cascade"] = QString::fromWCharArray(L"\x1F568");
    entypo_character_map_["flow-branch"] = QString::fromWCharArray(L"\x1F569");
    entypo_character_map_["flow-tree"] = QString::fromWCharArray(L"\x1F56A");
    entypo_character_map_["flow-line"] = QString::fromWCharArray(L"\x1F56B");
    entypo_character_map_["flow-parallel"] = QString::fromWCharArray(L"\x1F56C");
    entypo_character_map_["left-bold"] = QString::fromWCharArray(L"\xE4AD");
    entypo_character_map_["down-bold"] = QString::fromWCharArray(L"\xE4B0");
    entypo_character_map_["up-bold"] = QString::fromWCharArray(L"\xE4AF");
    entypo_character_map_["right-bold"] = QString::fromWCharArray(L"\xE4AE");
    entypo_character_map_["left"] = QString::fromWCharArray(L"\x2B05");
    entypo_character_map_["down"] = QString::fromWCharArray(L"\x2B07");
    entypo_character_map_["up"] = QString::fromWCharArray(L"\x2B06");
    entypo_character_map_["right"] = QString::fromWCharArray(L"\x27A1");
    entypo_character_map_["circled-left"] = QString::fromWCharArray(L"\xE759");
    entypo_character_map_["circled-down"] = QString::fromWCharArray(L"\xE758");
    entypo_character_map_["circled-up"] = QString::fromWCharArray(L"\xE75B");
    entypo_character_map_["circled-right"] = QString::fromWCharArray(L"\xE75A");
    entypo_character_map_["triangle-left"] = QString::fromWCharArray(L"\x25C2");
    entypo_character_map_["triangle-down"] = QString::fromWCharArray(L"\x25BE");
    entypo_character_map_["triangle-up"] = QString::fromWCharArray(L"\x25B4");
    entypo_character_map_["triangle-right"] = QString::fromWCharArray(L"\x25B8");
    entypo_character_map_["chevron-left"] = QString::fromWCharArray(L"\xE75D");
    entypo_character_map_["chevron-down"] = QString::fromWCharArray(L"\xE75C");
    entypo_character_map_["chevron-up"] = QString::fromWCharArray(L"\xE75F");
    entypo_character_map_["chevron-right"] = QString::fromWCharArray(L"\xE75E");
    entypo_character_map_["chevron-small-left"] = QString::fromWCharArray(L"\xE761");
    entypo_character_map_["chevron-small-down"] = QString::fromWCharArray(L"\xE760");
    entypo_character_map_["chevron-small-up"] = QString::fromWCharArray(L"\xE763");
    entypo_character_map_["chevron-small-right"] = QString::fromWCharArray(L"\xE762");
    entypo_character_map_["chevron-thin-left"] = QString::fromWCharArray(L"\xE765");
    entypo_character_map_["chevron-thin-down"] = QString::fromWCharArray(L"\xE764");
    entypo_character_map_["chevron-thin-up"] = QString::fromWCharArray(L"\xE767");
    entypo_character_map_["chevron-thin-right"] = QString::fromWCharArray(L"\xE766");
    entypo_character_map_["left-thin"] = QString::fromWCharArray(L"\x2190");
    entypo_character_map_["down-thin"] = QString::fromWCharArray(L"\x2193");
    entypo_character_map_["up-thin"] = QString::fromWCharArray(L"\x2191");
    entypo_character_map_["right-thin"] = QString::fromWCharArray(L"\x2192");
    entypo_character_map_["arrow-combo"] = QString::fromWCharArray(L"\xE74F");
    entypo_character_map_["three-dots"] = QString::fromWCharArray(L"\x23F6");
    entypo_character_map_["two-dots"] = QString::fromWCharArray(L"\x23F5");
    entypo_character_map_["dot"] = QString::fromWCharArray(L"\x23F4");
    entypo_character_map_["cc"] = QString::fromWCharArray(L"\x1F545");
    entypo_character_map_["cc-by"] = QString::fromWCharArray(L"\x1F546");
    entypo_character_map_["cc-nc"] = QString::fromWCharArray(L"\x1F547");
    entypo_character_map_["cc-nc-eu"] = QString::fromWCharArray(L"\x1F548");
    entypo_character_map_["cc-nc-jp"] = QString::fromWCharArray(L"\x1F549");
    entypo_character_map_["cc-sa"] = QString::fromWCharArray(L"\x1F54A");
    entypo_character_map_["cc-nd"] = QString::fromWCharArray(L"\x1F54B");
    entypo_character_map_["cc-pd"] = QString::fromWCharArray(L"\x1F54C");
    entypo_character_map_["cc-zero"] = QString::fromWCharArray(L"\x1F54D");
    entypo_character_map_["cc-share"] = QString::fromWCharArray(L"\x1F54E");
    entypo_character_map_["cc-remix"] = QString::fromWCharArray(L"\x1F54F");
    entypo_character_map_["db-logo"] = QString::fromWCharArray(L"\x1F5F9");
    entypo_character_map_["db-shape"] = QString::fromWCharArray(L"\x1F5FA");

    entypo_social_character_map_["github"] = QString::fromWCharArray(L"\xF300");
    entypo_social_character_map_["c-github"] = QString::fromWCharArray(L"\xF301");
    entypo_social_character_map_["flickr"] = QString::fromWCharArray(L"\xF303");
    entypo_social_character_map_["c-flickr"] = QString::fromWCharArray(L"\xF304");
    entypo_social_character_map_["vimeo"] = QString::fromWCharArray(L"\xF306");
    entypo_social_character_map_["c-vimeo"] = QString::fromWCharArray(L"\xF307");
    entypo_social_character_map_["twitter"] = QString::fromWCharArray(L"\xF309");
    entypo_social_character_map_["c-twitter"] = QString::fromWCharArray(L"\xF30A");
    entypo_social_character_map_["facebook"] = QString::fromWCharArray(L"\xF30C");
    entypo_social_character_map_["c-facebook"] = QString::fromWCharArray(L"\xF30D");
    entypo_social_character_map_["s-facebook"] = QString::fromWCharArray(L"\xF30E");
    entypo_social_character_map_["google+"] = QString::fromWCharArray(L"\xF30F");
    entypo_social_character_map_["c-google+"] = QString::fromWCharArray(L"\xF310");
    entypo_social_character_map_["pinterest"] = QString::fromWCharArray(L"\xF312");
    entypo_social_character_map_["c-pinterest"] = QString::fromWCharArray(L"\xF313");
    entypo_social_character_map_["tumblr"] = QString::fromWCharArray(L"\xF315");
    entypo_social_character_map_["c-tumblr"] = QString::fromWCharArray(L"\xF316");
    entypo_social_character_map_["linkedin"] = QString::fromWCharArray(L"\xF318");
    entypo_social_character_map_["c-linkedin"] = QString::fromWCharArray(L"\xF319");
    entypo_social_character_map_["dribbble"] = QString::fromWCharArray(L"\xF31B");
    entypo_social_character_map_["c-dribbble"] = QString::fromWCharArray(L"\xF31C");
    entypo_social_character_map_["stumbleupon"] = QString::fromWCharArray(L"\xF31E");
    entypo_social_character_map_["c-stumbleupon"] = QString::fromWCharArray(L"\xF31F");
    entypo_social_character_map_["lastfm"] = QString::fromWCharArray(L"\xF321");
    entypo_social_character_map_["c-lastfm"] = QString::fromWCharArray(L"\xF322");
    entypo_social_character_map_["rdio"] = QString::fromWCharArray(L"\xF324");
    entypo_social_character_map_["c-rdio"] = QString::fromWCharArray(L"\xF325");
    entypo_social_character_map_["spotify"] = QString::fromWCharArray(L"\xF327");
    entypo_social_character_map_["c-spotify"] = QString::fromWCharArray(L"\xF328");
    entypo_social_character_map_["qq"] = QString::fromWCharArray(L"\xF32A");
    entypo_social_character_map_["instagram"] = QString::fromWCharArray(L"\xF32D");
    entypo_social_character_map_["dropbox"] = QString::fromWCharArray(L"\xF330");
    entypo_social_character_map_["evernote"] = QString::fromWCharArray(L"\xF333");
    entypo_social_character_map_["flattr"] = QString::fromWCharArray(L"\xF336");
    entypo_social_character_map_["skype"] = QString::fromWCharArray(L"\xF339");
    entypo_social_character_map_["c-skype"] = QString::fromWCharArray(L"\xF33A");
    entypo_social_character_map_["renren"] = QString::fromWCharArray(L"\xF33C");
    entypo_social_character_map_["sina-weibo"] = QString::fromWCharArray(L"\xF33F");
    entypo_social_character_map_["paypal"] = QString::fromWCharArray(L"\xF342");
    entypo_social_character_map_["picasa"] = QString::fromWCharArray(L"\xF345");
    entypo_social_character_map_["soundcloud"] = QString::fromWCharArray(L"\xF348");
    entypo_social_character_map_["mixi"] = QString::fromWCharArray(L"\xF34B");
    entypo_social_character_map_["behance"] = QString::fromWCharArray(L"\xF34E");
    entypo_social_character_map_["google-circles"] = QString::fromWCharArray(L"\xF351");
    entypo_social_character_map_["vk"] = QString::fromWCharArray(L"\xF354");
    entypo_social_character_map_["smashing"] = QString::fromWCharArray(L"\xF357");

  }
  
  bool PictogramObject::isCharacterSupported(std::string character)
  {
    return ((entypo_social_character_map_.find(character)
             != entypo_social_character_map_.end()) ||
            (entypo_character_map_.find(character)
             != entypo_character_map_.end()));
  }
  
  QFont PictogramObject::getFont(std::string character)
  {
    if (entypo_social_character_map_.find(character)
        != entypo_social_character_map_.end()) {
      return QFont("Entypo Social");
    }
    else {
      return QFont("Entypo");
    }
  }
  
  QString PictogramObject::lookupPictogramText(std::string character)
  {
    if (entypo_social_character_map_.find(character)
        != entypo_social_character_map_.end()) {
      return entypo_social_character_map_[character];
    }
    else {
      return entypo_character_map_[character];
    }
  }
  
  PictogramObject::PictogramObject(Ogre::SceneManager* manager,
                                   Ogre::SceneNode* parent,
                                   double size,
                                   int entypo_font_id,
                                   int entypo_social_font_id):
    FacingTexturedObject(manager, parent, size),
    entypo_font_id_(entypo_font_id),
    entypo_social_font_id_(entypo_social_font_id)
  {
    square_object_->setPolygonType(SquareObject::SQUARE);
    square_object_->rebuildPolygon();
    setupCharacterMap();
  }
  
  void PictogramObject::update(float wall_dt, float ros_dt)
  {
    ScopedPixelBuffer buffer = texture_object_->getBuffer();
    QColor transparent(255, 255, 255, 0);
    QColor white(25, 255, 240, 255);
    QImage Hud = buffer.getQImage(128, 128, transparent); // should change according to size
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);
    QColor foreground = rviz::ogreToQt(color_);
    painter.setPen(QPen(foreground, 5, Qt::SolidLine));
    painter.setBrush(white);
    //QFont font("DejaVu Sans Mono");
    if (isCharacterSupported(text_)) {
      QFont font = getFont(text_);
      font.setPointSize(200);
      painter.setFont(font);
      painter.drawText(0, 0, 128, 128,
                       Qt::AlignHCenter | Qt::AlignVCenter,
                       lookupPictogramText(text_));
      painter.end();
    }
    else {
      ROS_WARN("%s is not supported", text_.c_str());
    }
  }

  void PictogramObject::updateColor()
  {
  }
  void PictogramObject::updateText()
  {
  }

  int PictogramDisplay::addFont(unsigned char* data, unsigned int data_len)
  {
    // register font
    QByteArray entypo =
      QByteArray::fromRawData(
        reinterpret_cast<const char*>(data), data_len);
    int id =
      QFontDatabase::addApplicationFontFromData(entypo);
    if (id == -1) {
      ROS_WARN("failed to load font");
    }
    else {
      return id;
    }

  }
  
  PictogramDisplay::PictogramDisplay()
  {
    entypo_id_ = addFont(Entypo_ttf, Entypo_ttf_len);
    entypo_social_id_ = addFont(Entypo_Social_ttf, Entypo_Social_ttf_len);
  }

  PictogramDisplay::~PictogramDisplay()
  {
    
  }
  
  void PictogramDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    pictogram_.reset(new PictogramObject(scene_manager_,
                                         scene_node_,
                                         1.0,
                                         entypo_id_,
                                         entypo_social_id_));
    pictogram_->setEnable(false);
    // initial setting
    pictogram_->setColor(QColor(25, 255, 240));
    pictogram_->setAlpha(1.0);
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  void PictogramDisplay::reset()
  {
    MFDClass::reset();
  }

  void PictogramDisplay::onEnable()
  {
    subscribe();
    if (pictogram_) {
      // keep false, it will be true
      // in side of processMessae callback.
      pictogram_->setEnable(false);
    }
  }

  void PictogramDisplay::processMessage(const jsk_rviz_plugins::Pictogram::ConstPtr& msg)
  {
    boost::mutex::scoped_lock (mutex_);
    pictogram_->setEnable(isEnabled());
    if (!isEnabled()) {
      return;
    }
    Ogre::Vector3 position;
    Ogre::Quaternion quaternion;
    if(!context_->getFrameManager()->transform(msg->header,
                                               msg->pose,
                                               position,
                                               quaternion)) {
      ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                 qPrintable( getName() ), msg->header.frame_id.c_str(),
                 qPrintable( fixed_frame_ ));
      return;
    }
    if (msg->size <= 0.0) {
      pictogram_->setSize(0.5);
    }
    else {
      pictogram_->setSize(msg->size);
    }
    pictogram_->setPosition(position);
    pictogram_->setOrientation(quaternion);
    pictogram_->setText(msg->character);
  }

  void PictogramDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock (mutex_);
    if (pictogram_) {
      pictogram_->update(wall_dt, ros_dt);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugin::PictogramDisplay, rviz::Display);
