<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.6.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="16" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="14" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="57" name="tCAD" color="7" fill="1" visible="no" active="no"/>
<layer number="59" name="tCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="60" name="bCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="no" active="no"/>
<layer number="101" name="Patch_Top" color="12" fill="4" visible="no" active="yes"/>
<layer number="102" name="Vscore" color="7" fill="1" visible="no" active="yes"/>
<layer number="103" name="tMap" color="7" fill="1" visible="no" active="yes"/>
<layer number="104" name="Name" color="16" fill="1" visible="no" active="yes"/>
<layer number="105" name="tPlate" color="7" fill="1" visible="no" active="yes"/>
<layer number="106" name="bPlate" color="7" fill="1" visible="no" active="yes"/>
<layer number="107" name="Crop" color="7" fill="1" visible="no" active="yes"/>
<layer number="108" name="tplace-old" color="10" fill="1" visible="yes" active="yes"/>
<layer number="109" name="ref-old" color="11" fill="1" visible="yes" active="yes"/>
<layer number="110" name="fp0" color="7" fill="1" visible="yes" active="yes"/>
<layer number="111" name="LPC17xx" color="7" fill="1" visible="yes" active="yes"/>
<layer number="112" name="tSilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="113" name="IDFDebug" color="7" fill="1" visible="yes" active="yes"/>
<layer number="114" name="Badge_Outline" color="7" fill="1" visible="yes" active="yes"/>
<layer number="115" name="ReferenceISLANDS" color="7" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="no" active="yes"/>
<layer number="118" name="Rect_Pads" color="7" fill="1" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="no" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="no" active="yes"/>
<layer number="123" name="tTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="124" name="bTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="125" name="_tNames" color="7" fill="1" visible="no" active="yes"/>
<layer number="126" name="_bNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="127" name="_tValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="128" name="_bValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="129" name="Mask" color="7" fill="1" visible="yes" active="yes"/>
<layer number="131" name="tAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="132" name="bAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="144" name="Drill_legend" color="7" fill="1" visible="no" active="yes"/>
<layer number="150" name="Notes" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="no" active="yes"/>
<layer number="152" name="_bDocu" color="7" fill="1" visible="yes" active="yes"/>
<layer number="153" name="FabDoc1" color="7" fill="1" visible="yes" active="yes"/>
<layer number="154" name="FabDoc2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="155" name="FabDoc3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="199" name="Contour" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="no" active="yes"/>
<layer number="201" name="201bmp" color="2" fill="10" visible="no" active="yes"/>
<layer number="202" name="202bmp" color="3" fill="10" visible="no" active="yes"/>
<layer number="203" name="203bmp" color="4" fill="10" visible="no" active="yes"/>
<layer number="204" name="204bmp" color="5" fill="10" visible="no" active="yes"/>
<layer number="205" name="205bmp" color="6" fill="10" visible="no" active="yes"/>
<layer number="206" name="206bmp" color="7" fill="10" visible="no" active="yes"/>
<layer number="207" name="207bmp" color="8" fill="10" visible="no" active="yes"/>
<layer number="208" name="208bmp" color="9" fill="10" visible="no" active="yes"/>
<layer number="209" name="209bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="210" name="210bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="211" name="211bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="212" name="212bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="213" name="213bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="214" name="214bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="215" name="215bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="216" name="216bmp" color="7" fill="1" visible="no" active="yes"/>
<layer number="217" name="217bmp" color="18" fill="1" visible="no" active="no"/>
<layer number="218" name="218bmp" color="19" fill="1" visible="no" active="no"/>
<layer number="219" name="219bmp" color="20" fill="1" visible="no" active="no"/>
<layer number="220" name="220bmp" color="21" fill="1" visible="no" active="no"/>
<layer number="221" name="221bmp" color="22" fill="1" visible="no" active="no"/>
<layer number="222" name="222bmp" color="23" fill="1" visible="no" active="no"/>
<layer number="223" name="223bmp" color="24" fill="1" visible="no" active="no"/>
<layer number="224" name="224bmp" color="25" fill="1" visible="no" active="no"/>
<layer number="225" name="225bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="226" name="226bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="227" name="227bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="228" name="228bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="229" name="229bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="230" name="230bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="231" name="231bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="232" name="Eagle3D_PG2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="233" name="Eagle3D_PG3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="248" name="Housing" color="7" fill="1" visible="yes" active="yes"/>
<layer number="249" name="Edge" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="cooling" color="7" fill="1" visible="no" active="yes"/>
<layer number="255" name="routoute" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="SparkFun-Capacitors">
<description>&lt;h3&gt;SparkFun Capacitors&lt;/h3&gt;
This library contains capacitors. 
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="0603">
<description>&lt;p&gt;&lt;b&gt;Generic 1608 (0603) package&lt;/b&gt;&lt;/p&gt;
&lt;p&gt;0.2mm courtyard excess rounded to nearest 0.05mm.&lt;/p&gt;</description>
<wire x1="-1.6" y1="0.7" x2="1.6" y2="0.7" width="0.0508" layer="39"/>
<wire x1="1.6" y1="0.7" x2="1.6" y2="-0.7" width="0.0508" layer="39"/>
<wire x1="1.6" y1="-0.7" x2="-1.6" y2="-0.7" width="0.0508" layer="39"/>
<wire x1="-1.6" y1="-0.7" x2="-1.6" y2="0.7" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="0" y="0.762" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-0.762" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="0402">
<description>&lt;p&gt;&lt;b&gt;Generic 1005 (0402) package&lt;/b&gt;&lt;/p&gt;
&lt;p&gt;0.2mm courtyard excess rounded to nearest 0.05mm.&lt;/p&gt;</description>
<wire x1="-0.2704" y1="0.2286" x2="0.2704" y2="0.2286" width="0.1524" layer="51"/>
<wire x1="0.2704" y1="-0.2286" x2="-0.2704" y2="-0.2286" width="0.1524" layer="51"/>
<wire x1="-1.2" y1="0.65" x2="1.2" y2="0.65" width="0.0508" layer="39"/>
<wire x1="1.2" y1="0.65" x2="1.2" y2="-0.65" width="0.0508" layer="39"/>
<wire x1="1.2" y1="-0.65" x2="-1.2" y2="-0.65" width="0.0508" layer="39"/>
<wire x1="-1.2" y1="-0.65" x2="-1.2" y2="0.65" width="0.0508" layer="39"/>
<smd name="1" x="-0.58" y="0" dx="0.85" dy="0.9" layer="1"/>
<smd name="2" x="0.58" y="0" dx="0.85" dy="0.9" layer="1"/>
<text x="0" y="0.762" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-0.762" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.3048" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="0805">
<description>&lt;p&gt;&lt;b&gt;Generic 2012 (0805) package&lt;/b&gt;&lt;/p&gt;
&lt;p&gt;0.2mm courtyard excess rounded to nearest 0.05mm.&lt;/p&gt;</description>
<smd name="1" x="-0.9" y="0" dx="0.8" dy="1.2" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.8" dy="1.2" layer="1"/>
<text x="0" y="0.889" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-0.889" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<wire x1="-1.5" y1="0.8" x2="1.5" y2="0.8" width="0.0508" layer="39"/>
<wire x1="1.5" y1="0.8" x2="1.5" y2="-0.8" width="0.0508" layer="39"/>
<wire x1="1.5" y1="-0.8" x2="-1.5" y2="-0.8" width="0.0508" layer="39"/>
<wire x1="-1.5" y1="-0.8" x2="-1.5" y2="0.8" width="0.0508" layer="39"/>
</package>
<package name="1206">
<description>&lt;p&gt;&lt;b&gt;Generic 3216 (1206) package&lt;/b&gt;&lt;/p&gt;
&lt;p&gt;0.2mm courtyard excess rounded to nearest 0.05mm.&lt;/p&gt;</description>
<wire x1="-2.4" y1="1.1" x2="2.4" y2="1.1" width="0.0508" layer="39"/>
<wire x1="2.4" y1="-1.1" x2="-2.4" y2="-1.1" width="0.0508" layer="39"/>
<wire x1="-2.4" y1="-1.1" x2="-2.4" y2="1.1" width="0.0508" layer="39"/>
<wire x1="2.4" y1="1.1" x2="2.4" y2="-1.1" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="0" y="1.143" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.143" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
</packages>
<symbols>
<symbol name="CAP">
<wire x1="0" y1="2.54" x2="0" y2="2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="0.508" width="0.1524" layer="94"/>
<text x="1.524" y="2.921" size="1.778" layer="95" font="vector">&gt;NAME</text>
<text x="1.524" y="-2.159" size="1.778" layer="96" font="vector">&gt;VALUE</text>
<rectangle x1="-2.032" y1="0.508" x2="2.032" y2="1.016" layer="94"/>
<rectangle x1="-2.032" y1="1.524" x2="2.032" y2="2.032" layer="94"/>
<pin name="1" x="0" y="5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="1.0UF" prefix="C">
<description>&lt;h3&gt;1µF ceramic capacitors&lt;/h3&gt;
&lt;p&gt;A capacitor is a passive two-terminal electrical component used to store electrical energy temporarily in an electric field.&lt;/p&gt;</description>
<gates>
<gate name="G$1" symbol="CAP" x="0" y="0"/>
</gates>
<devices>
<device name="-0603-16V-10%" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-00868"/>
<attribute name="VALUE" value="1.0uF"/>
</technology>
</technologies>
</device>
<device name="-0402-16V-10%" package="0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-12417"/>
<attribute name="VALUE" value="1.0uF"/>
</technology>
</technologies>
</device>
<device name="-0805-25V-(+80/-20%)" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-11625"/>
<attribute name="VALUE" value="1.0uF"/>
</technology>
</technologies>
</device>
<device name="-1206-50V-10%" package="1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-09822"/>
<attribute name="VALUE" value="1.0uF"/>
</technology>
</technologies>
</device>
<device name="-0805-25V-10%" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-08064"/>
<attribute name="VALUE" value="1.0uF"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-Switches">
<description>&lt;h3&gt;SparkFun Switches, Buttons, Encoders&lt;/h3&gt;
In this library you'll find switches, buttons, joysticks, and anything that moves to create or disrupt an electrical connection.
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="REED_SWITCH_GLASS">
<description>&lt;h3&gt;Reed Switch - Glass case - PTH&lt;/h3&gt;
&lt;p&gt;A reed switch is a magnetically-actuated switch. When the device is exposed to a magnetic field, the two ferrous materials inside the switch pull together and the switch closes.&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.sparkfun.com/datasheets/Components/Buttons/MDSR-4.pdf"&gt;Datasheet&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="-6.985" y1="-1.143" x2="6.985" y2="-1.143" width="0.2032" layer="21"/>
<wire x1="-6.985" y1="-1.143" x2="-6.985" y2="0" width="0.2032" layer="21"/>
<wire x1="-6.985" y1="0" x2="-6.985" y2="1.143" width="0.2032" layer="21"/>
<wire x1="-6.985" y1="1.143" x2="6.985" y2="1.143" width="0.2032" layer="21"/>
<wire x1="6.985" y1="1.143" x2="6.985" y2="0" width="0.2032" layer="21"/>
<wire x1="6.985" y1="0" x2="6.985" y2="-1.143" width="0.2032" layer="21"/>
<wire x1="-6.985" y1="0" x2="-7.62" y2="0" width="0.2032" layer="21"/>
<wire x1="6.985" y1="0" x2="7.62" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-8.89" y="0" drill="1.016" diameter="1.8796"/>
<pad name="P$2" x="8.89" y="0" drill="1.016" diameter="1.8796"/>
<text x="0" y="1.27" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.27" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="REED_SWITCH_PLASTIC">
<description>&lt;h3&gt;Reed Switch - Insulated Case - PTH&lt;/h3&gt;
&lt;p&gt;A reed switch is a magnetically-actuated switch. When the device is exposed to a magnetic field, the two ferrous materials inside the switch pull together and the switch closes.&lt;/p&gt;
&lt;p&gt;&lt;a href="https://cdn.sparkfun.com/datasheets/Dev/LilyPad/RS-01C.jpg"&gt;Datasheet&lt;/a&gt; (RS-01C)&lt;/p&gt;</description>
<wire x1="-7.5" y1="-1.65" x2="7.5" y2="-1.65" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="-1.65" x2="-7.5" y2="0" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="0" x2="-7.5" y2="1.65" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="1.65" x2="7.5" y2="1.65" width="0.2032" layer="21"/>
<wire x1="7.5" y1="1.65" x2="7.5" y2="0" width="0.2032" layer="21"/>
<wire x1="7.5" y1="0" x2="7.5" y2="-1.65" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="0" x2="-7.72" y2="0" width="0.2032" layer="21"/>
<wire x1="7.5" y1="0" x2="7.72" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-8.89" y="0" drill="1.016" diameter="1.8796"/>
<pad name="P$2" x="8.89" y="0" drill="1.016" diameter="1.8796"/>
<text x="0" y="1.778" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.778" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_PTH_6.0MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - PTH, 6.0mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.omron.com/ecb/products/pdf/en-b3f.pdf"&gt;Datasheet&lt;/a&gt; (B3F-1000)&lt;/p&gt;</description>
<wire x1="3.048" y1="1.016" x2="3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="2.54" x2="2.54" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="3.048" x2="2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.159" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="21"/>
<wire x1="-2.159" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="21"/>
<wire x1="3.048" y1="0.998" x2="3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-3.048" y1="1.028" x2="-3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="0.508" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="-2.54" y2="-1.27" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="-2.159" y2="-0.381" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.778" width="0.2032" layer="21"/>
<pad name="1" x="-3.2512" y="2.2606" drill="1.016" diameter="1.8796"/>
<pad name="2" x="3.2512" y="2.2606" drill="1.016" diameter="1.8796"/>
<pad name="3" x="-3.2512" y="-2.2606" drill="1.016" diameter="1.8796"/>
<pad name="4" x="3.2512" y="-2.2606" drill="1.016" diameter="1.8796"/>
<text x="0" y="3.302" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-3.175" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_4.5MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 4.5mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="http://spec_sheets.e-switch.com/specs/P010338.pdf"&gt;Dimensional Drawing&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="1.905" y1="1.27" x2="1.905" y2="0.445" width="0.127" layer="51"/>
<wire x1="1.905" y1="0.445" x2="2.16" y2="-0.01" width="0.127" layer="51"/>
<wire x1="1.905" y1="-0.23" x2="1.905" y2="-1.115" width="0.127" layer="51"/>
<wire x1="-2.25" y1="2.25" x2="2.25" y2="2.25" width="0.127" layer="51"/>
<wire x1="2.25" y1="2.25" x2="2.25" y2="-2.25" width="0.127" layer="51"/>
<wire x1="2.25" y1="-2.25" x2="-2.25" y2="-2.25" width="0.127" layer="51"/>
<wire x1="-2.25" y1="-2.25" x2="-2.25" y2="2.25" width="0.127" layer="51"/>
<wire x1="-2.2" y1="0.8" x2="-2.2" y2="-0.8" width="0.2032" layer="21"/>
<wire x1="1.3" y1="2.2" x2="-1.3" y2="2.2" width="0.2032" layer="21"/>
<wire x1="2.2" y1="-0.8" x2="2.2" y2="0.8" width="0.2032" layer="21"/>
<wire x1="-1.3" y1="-2.2" x2="1.3" y2="-2.2" width="0.2032" layer="21"/>
<wire x1="2.2" y1="0.8" x2="1.8" y2="0.8" width="0.2032" layer="21"/>
<wire x1="2.2" y1="-0.8" x2="1.8" y2="-0.8" width="0.2032" layer="21"/>
<wire x1="-1.8" y1="0.8" x2="-2.2" y2="0.8" width="0.2032" layer="21"/>
<wire x1="-1.8" y1="-0.8" x2="-2.2" y2="-0.8" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.2032" layer="21"/>
<smd name="1" x="2.225" y="1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="2" x="2.225" y="-1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="3" x="-2.225" y="-1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="4" x="-2.225" y="1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<text x="0" y="2.413" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-2.413" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_PTH_12MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - PTH, 12mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.omron.com/ecb/products/pdf/en-b3f.pdf"&gt;Datasheet&lt;/a&gt; (B3F-5050)&lt;/p&gt;</description>
<wire x1="5" y1="-1.3" x2="5" y2="-0.7" width="0.2032" layer="51"/>
<wire x1="5" y1="-0.7" x2="4.5" y2="-0.2" width="0.2032" layer="51"/>
<wire x1="5" y1="0.2" x2="5" y2="1" width="0.2032" layer="51"/>
<wire x1="-6" y1="4" x2="-6" y2="5" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="6" y2="4" width="0.2032" layer="21"/>
<wire x1="6" y1="1" x2="6" y2="-1" width="0.2032" layer="21"/>
<wire x1="6" y1="-4" x2="6" y2="-5" width="0.2032" layer="21"/>
<wire x1="5" y1="-6" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="-6" y1="-5" x2="-6" y2="-4" width="0.2032" layer="21"/>
<wire x1="-6" y1="-1" x2="-6" y2="1" width="0.2032" layer="21"/>
<wire x1="-6" y1="5" x2="-5" y2="6" width="0.2032" layer="21" curve="-90"/>
<wire x1="5" y1="6" x2="6" y2="5" width="0.2032" layer="21" curve="-90"/>
<wire x1="6" y1="-5" x2="5" y2="-6" width="0.2032" layer="21" curve="-90"/>
<wire x1="-5" y1="-6" x2="-6" y2="-5" width="0.2032" layer="21" curve="-90"/>
<circle x="0" y="0" radius="3.5" width="0.2032" layer="21"/>
<circle x="-4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="-4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<pad name="4" x="-6.25" y="2.5" drill="1.2" diameter="2.159"/>
<pad name="2" x="-6.25" y="-2.5" drill="1.2" diameter="2.159"/>
<pad name="1" x="6.25" y="-2.5" drill="1.2" diameter="2.159"/>
<pad name="3" x="6.25" y="2.5" drill="1.2" diameter="2.159"/>
<text x="0" y="6.223" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-6.223" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_6.0X3.5MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 6.0 x 3.5 mm&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.sparkfun.com/datasheets/Components/1101.pdf"&gt;Datasheet&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="-3" y1="1.1" x2="-3" y2="-1.1" width="0.127" layer="51"/>
<wire x1="3" y1="1.1" x2="3" y2="-1.1" width="0.127" layer="51"/>
<wire x1="-2.75" y1="1.75" x2="-3" y2="1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.75" y1="1.75" x2="2.75" y2="1.75" width="0.2032" layer="21"/>
<wire x1="2.75" y1="1.75" x2="3" y2="1.5" width="0.2032" layer="21" curve="-90"/>
<wire x1="3" y1="-1.5" x2="2.75" y2="-1.75" width="0.2032" layer="21" curve="-90"/>
<wire x1="2.75" y1="-1.75" x2="-2.75" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="-3" y1="-1.5" x2="-2.75" y2="-1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="-3" y1="-1.5" x2="-3" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-3" y1="1.1" x2="-3" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3" y1="1.1" x2="3" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3" y1="-1.5" x2="3" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-1.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="1.5" y1="-0.75" x2="-1.5" y2="-0.75" width="0.2032" layer="21"/>
<wire x1="-1.5" y1="-0.75" x2="-1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="1.5" y1="-0.75" x2="1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="-2" y1="0" x2="-1" y2="0" width="0.127" layer="51"/>
<wire x1="-1" y1="0" x2="0.1" y2="0.5" width="0.127" layer="51"/>
<wire x1="0.3" y1="0" x2="2" y2="0" width="0.127" layer="51"/>
<smd name="1" x="-3.15" y="0" dx="2.3" dy="1.6" layer="1" rot="R180"/>
<smd name="2" x="3.15" y="0" dx="2.3" dy="1.6" layer="1" rot="R180"/>
<text x="0" y="1.905" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.905" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_6.2MM_TALL">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 6.2mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="http://www.apem.com/files/apem/brochures/ADTS6-ADTSM-KTSC6.pdf"&gt;Datasheet&lt;/a&gt; (ADTSM63NVTR)&lt;/p&gt;</description>
<wire x1="-3" y1="-3" x2="3" y2="-3" width="0.2032" layer="21"/>
<wire x1="3" y1="-3" x2="3" y2="3" width="0.2032" layer="21"/>
<wire x1="3" y1="3" x2="-3" y2="3" width="0.2032" layer="21"/>
<wire x1="-3" y1="3" x2="-3" y2="-3" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="1.75" width="0.2032" layer="21"/>
<smd name="A1" x="-3.975" y="-2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="A2" x="3.975" y="-2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="B1" x="-3.975" y="2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="B2" x="3.975" y="2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<text x="0" y="3.175" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-3.175" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_PTH_RIGHT_ANGLE_KIT">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - PTH, Right-angle&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/Switches/SW016.JPG"&gt;Dimensional Drawing&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="1.5" y1="-3.8" x2="-1.5" y2="-3.8" width="0.127" layer="51"/>
<wire x1="-3.65" y1="-2" x2="-3.65" y2="3.5" width="0.127" layer="51"/>
<wire x1="-3.65" y1="3.5" x2="-3" y2="3.5" width="0.127" layer="51"/>
<wire x1="3" y1="3.5" x2="3.65" y2="3.5" width="0.127" layer="51"/>
<wire x1="3.65" y1="3.5" x2="3.65" y2="-2" width="0.127" layer="51"/>
<wire x1="-3" y1="2" x2="3" y2="2" width="0.127" layer="51"/>
<wire x1="-3" y1="2" x2="-3" y2="3.5" width="0.127" layer="51"/>
<wire x1="3" y1="2" x2="3" y2="3.5" width="0.127" layer="51"/>
<wire x1="-3.65" y1="-2" x2="-1.5" y2="-2" width="0.127" layer="51"/>
<wire x1="-1.5" y1="-2" x2="1.5" y2="-2" width="0.127" layer="51"/>
<wire x1="1.5" y1="-2" x2="3.65" y2="-2" width="0.127" layer="51"/>
<wire x1="1.5" y1="-2" x2="1.5" y2="-3.8" width="0.127" layer="51"/>
<wire x1="-1.5" y1="-2" x2="-1.5" y2="-3.8" width="0.127" layer="51"/>
<wire x1="-3.777" y1="1" x2="-3.777" y2="-2.127" width="0.2032" layer="21"/>
<wire x1="-3.777" y1="-2.127" x2="3.777" y2="-2.127" width="0.2032" layer="21"/>
<wire x1="3.777" y1="-2.127" x2="3.777" y2="1" width="0.2032" layer="21"/>
<wire x1="2" y1="2.127" x2="-2" y2="2.127" width="0.2032" layer="21"/>
<pad name="ANCHOR1" x="-3.5" y="2.5" drill="1.2" diameter="2.2" stop="no"/>
<pad name="ANCHOR2" x="3.5" y="2.5" drill="1.2" diameter="2.2" stop="no"/>
<pad name="1" x="-2.5" y="0" drill="0.8" diameter="1.7" stop="no"/>
<pad name="2" x="2.5" y="0" drill="0.8" diameter="1.7" stop="no"/>
<circle x="2.5" y="0" radius="0.4445" width="0" layer="29"/>
<circle x="-2.5" y="0" radius="0.4445" width="0" layer="29"/>
<circle x="-3.5" y="2.5" radius="0.635" width="0" layer="29"/>
<circle x="3.5" y="2.5" radius="0.635" width="0" layer="29"/>
<circle x="-3.5" y="2.5" radius="1.143" width="0" layer="30"/>
<circle x="2.5" y="0" radius="0.889" width="0" layer="30"/>
<circle x="-2.5" y="0" radius="0.889" width="0" layer="30"/>
<circle x="3.5" y="2.5" radius="1.143" width="0" layer="30"/>
<text x="0" y="2.286" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-2.286" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_12MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 12mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="https://cdn.sparkfun.com/datasheets/Components/Switches/N301102.pdf"&gt;Datasheet&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="5" y1="-1.3" x2="5" y2="-0.7" width="0.2032" layer="51"/>
<wire x1="5" y1="-0.7" x2="4.5" y2="-0.2" width="0.2032" layer="51"/>
<wire x1="5" y1="0.2" x2="5" y2="1" width="0.2032" layer="51"/>
<wire x1="-6" y1="4" x2="-6" y2="5" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="6" y2="4" width="0.2032" layer="21"/>
<wire x1="6" y1="1" x2="6" y2="-1" width="0.2032" layer="21"/>
<wire x1="6" y1="-4" x2="6" y2="-5" width="0.2032" layer="21"/>
<wire x1="5" y1="-6" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="-6" y1="-5" x2="-6" y2="-4" width="0.2032" layer="21"/>
<wire x1="-6" y1="-1" x2="-6" y2="1" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="3.5" width="0.2032" layer="21"/>
<circle x="-4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="-4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<smd name="4" x="-6.975" y="2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="2" x="-6.975" y="-2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="1" x="6.975" y="-2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="3" x="6.975" y="2.5" dx="1.6" dy="1.55" layer="1"/>
<wire x1="-6" y1="-5" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="6" y1="-5" x2="5" y2="-6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="-6" y2="5" width="0.2032" layer="21"/>
<text x="0" y="6.223" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-6.223" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_PTH_6.0MM_KIT">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - PTH, 6.0mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;b&gt;Warning:&lt;/b&gt; This is the KIT version of this package. This package has a smaller diameter top stop mask, which doesn't cover the diameter of the pad. This means only the bottom side of the pads' copper will be exposed. You'll only be able to solder to the bottom side.&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.omron.com/ecb/products/pdf/en-b3f.pdf"&gt;Datasheet&lt;/a&gt; (B3F-1000)&lt;/p&gt;</description>
<wire x1="3.048" y1="1.016" x2="3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="2.54" x2="2.54" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="3.048" x2="2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.159" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="21"/>
<wire x1="-2.159" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="21"/>
<wire x1="3.048" y1="0.998" x2="3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-3.048" y1="1.028" x2="-3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="0.508" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="-2.54" y2="-1.27" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="-2.159" y2="-0.381" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.778" width="0.2032" layer="21"/>
<pad name="1" x="-3.2512" y="2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="2" x="3.2512" y="2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="3" x="-3.2512" y="-2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="4" x="3.2512" y="-2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<polygon width="0.127" layer="30">
<vertex x="-3.2664" y="3.142"/>
<vertex x="-3.2589" y="3.1445" curve="89.986886"/>
<vertex x="-4.1326" y="2.286"/>
<vertex x="-4.1351" y="2.2657" curve="90.00652"/>
<vertex x="-3.2563" y="1.392"/>
<vertex x="-3.2487" y="1.3869" curve="90.006616"/>
<vertex x="-2.3826" y="2.2403"/>
<vertex x="-2.3775" y="2.2683" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-3.2462" y="2.7026"/>
<vertex x="-3.2589" y="2.7051" curve="90.026544"/>
<vertex x="-3.6881" y="2.2733"/>
<vertex x="-3.6881" y="2.2632" curve="89.974074"/>
<vertex x="-3.2562" y="1.8213"/>
<vertex x="-3.2259" y="1.8186" curve="90.051271"/>
<vertex x="-2.8093" y="2.2658"/>
<vertex x="-2.8093" y="2.2606" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="3.2411" y="3.1395"/>
<vertex x="3.2486" y="3.142" curve="89.986886"/>
<vertex x="2.3749" y="2.2835"/>
<vertex x="2.3724" y="2.2632" curve="90.00652"/>
<vertex x="3.2512" y="1.3895"/>
<vertex x="3.2588" y="1.3844" curve="90.006616"/>
<vertex x="4.1249" y="2.2378"/>
<vertex x="4.13" y="2.2658" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="3.2613" y="2.7001"/>
<vertex x="3.2486" y="2.7026" curve="90.026544"/>
<vertex x="2.8194" y="2.2708"/>
<vertex x="2.8194" y="2.2607" curve="89.974074"/>
<vertex x="3.2513" y="1.8188"/>
<vertex x="3.2816" y="1.8161" curve="90.051271"/>
<vertex x="3.6982" y="2.2633"/>
<vertex x="3.6982" y="2.2581" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="-3.2613" y="-1.3868"/>
<vertex x="-3.2538" y="-1.3843" curve="89.986886"/>
<vertex x="-4.1275" y="-2.2428"/>
<vertex x="-4.13" y="-2.2631" curve="90.00652"/>
<vertex x="-3.2512" y="-3.1368"/>
<vertex x="-3.2436" y="-3.1419" curve="90.006616"/>
<vertex x="-2.3775" y="-2.2885"/>
<vertex x="-2.3724" y="-2.2605" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-3.2411" y="-1.8262"/>
<vertex x="-3.2538" y="-1.8237" curve="90.026544"/>
<vertex x="-3.683" y="-2.2555"/>
<vertex x="-3.683" y="-2.2656" curve="89.974074"/>
<vertex x="-3.2511" y="-2.7075"/>
<vertex x="-3.2208" y="-2.7102" curve="90.051271"/>
<vertex x="-2.8042" y="-2.263"/>
<vertex x="-2.8042" y="-2.2682" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="3.2411" y="-1.3843"/>
<vertex x="3.2486" y="-1.3818" curve="89.986886"/>
<vertex x="2.3749" y="-2.2403"/>
<vertex x="2.3724" y="-2.2606" curve="90.00652"/>
<vertex x="3.2512" y="-3.1343"/>
<vertex x="3.2588" y="-3.1394" curve="90.006616"/>
<vertex x="4.1249" y="-2.286"/>
<vertex x="4.13" y="-2.258" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="3.2613" y="-1.8237"/>
<vertex x="3.2486" y="-1.8212" curve="90.026544"/>
<vertex x="2.8194" y="-2.253"/>
<vertex x="2.8194" y="-2.2631" curve="89.974074"/>
<vertex x="3.2513" y="-2.705"/>
<vertex x="3.2816" y="-2.7077" curve="90.051271"/>
<vertex x="3.6982" y="-2.2605"/>
<vertex x="3.6982" y="-2.2657" curve="90.012964"/>
</polygon>
<text x="0" y="3.175" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-3.175" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_5.2MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 5.2mm Square&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="https://www.sparkfun.com/datasheets/Components/Buttons/SMD-Button.pdf"&gt;Dimensional Drawing&lt;/a&gt;&lt;/p&gt;</description>
<wire x1="-1.54" y1="-2.54" x2="-2.54" y2="-1.54" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-1.24" x2="-2.54" y2="1.27" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.54" x2="-1.54" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="2.54" x2="1.54" y2="2.54" width="0.2032" layer="21"/>
<wire x1="1.54" y1="2.54" x2="2.54" y2="1.54" width="0.2032" layer="51"/>
<wire x1="2.54" y1="1.24" x2="2.54" y2="-1.24" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-1.54" x2="1.54" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="1.54" y1="-2.54" x2="-1.54" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="1.905" y2="0.445" width="0.127" layer="51"/>
<wire x1="1.905" y1="0.445" x2="2.16" y2="-0.01" width="0.127" layer="51"/>
<wire x1="1.905" y1="-0.23" x2="1.905" y2="-1.115" width="0.127" layer="51"/>
<circle x="0" y="0" radius="1.27" width="0.2032" layer="21"/>
<smd name="1" x="-2.794" y="1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="2" x="2.794" y="1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="3" x="-2.794" y="-1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="4" x="2.794" y="-1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<text x="0" y="2.667" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-2.667" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_RIGHT_ANGLE">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, Right-angle&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;</description>
<hole x="0" y="0.9" drill="0.7"/>
<hole x="0" y="-0.9" drill="0.7"/>
<smd name="1" x="-1.95" y="0" dx="2" dy="1.1" layer="1" rot="R90"/>
<smd name="2" x="1.95" y="0" dx="2" dy="1.1" layer="1" rot="R90"/>
<wire x1="-2" y1="1.2" x2="-2" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-2" y1="1.5" x2="2" y2="1.5" width="0.2032" layer="21"/>
<wire x1="2" y1="1.5" x2="2" y2="1.2" width="0.2032" layer="21"/>
<wire x1="-2" y1="-1.2" x2="-2" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-2" y1="-1.5" x2="-0.7" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-0.7" y1="-1.5" x2="0.7" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="0.7" y1="-1.5" x2="2" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="2" y1="-1.5" x2="2" y2="-1.2" width="0.2032" layer="21"/>
<wire x1="-0.7" y1="-2.1" x2="0.7" y2="-2.1" width="0.2032" layer="21"/>
<wire x1="0.7" y1="-2.1" x2="0.7" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-0.7" y1="-2.1" x2="-0.7" y2="-1.5" width="0.2032" layer="21"/>
<text x="0" y="1.651" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-2.286" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH_SMD_4.6X2.8MM">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST - SMD, 4.6 x 2.8mm&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;p&gt;&lt;a href="http://www.ck-components.com/media/1479/kmr2.pdf"&gt;Datasheet&lt;/a&gt;&lt;/p&gt;</description>
<smd name="3" x="2.05" y="0.8" dx="0.9" dy="1" layer="1"/>
<smd name="2" x="2.05" y="-0.8" dx="0.9" dy="1" layer="1"/>
<smd name="1" x="-2.05" y="-0.8" dx="0.9" dy="1" layer="1"/>
<smd name="4" x="-2.05" y="0.8" dx="0.9" dy="1" layer="1"/>
<wire x1="-2.1" y1="1.4" x2="-2.1" y2="-1.4" width="0.127" layer="51"/>
<wire x1="2.1" y1="-1.4" x2="2.1" y2="1.4" width="0.127" layer="51"/>
<wire x1="-2.1" y1="1.4" x2="2.1" y2="1.4" width="0.127" layer="51"/>
<wire x1="-2.1" y1="-1.4" x2="2.1" y2="-1.4" width="0.127" layer="51"/>
<circle x="0" y="0" radius="0.805" width="0.127" layer="21"/>
<wire x1="1.338" y1="-1.4" x2="-1.338" y2="-1.4" width="0.2032" layer="21"/>
<wire x1="-1.338" y1="1.4" x2="1.338" y2="1.4" width="0.2032" layer="21"/>
<wire x1="-2.1" y1="0.13" x2="-2.1" y2="-0.13" width="0.2032" layer="21"/>
<wire x1="2.1" y1="-0.13" x2="2.1" y2="0.13" width="0.2032" layer="21"/>
<rectangle x1="-2.3" y1="0.5" x2="-2.1" y2="1.1" layer="51"/>
<rectangle x1="-2.3" y1="-1.1" x2="-2.1" y2="-0.5" layer="51"/>
<rectangle x1="2.1" y1="-1.1" x2="2.3" y2="-0.5" layer="51" rot="R180"/>
<rectangle x1="2.1" y1="0.5" x2="2.3" y2="1.1" layer="51" rot="R180"/>
<text x="0" y="1.524" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.524" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="ROTARY_ENC_PLAIN">
<description>&lt;h3&gt;Rotary Encoder w/ Select Switch&lt;/h3&gt;
&lt;p&gt;Encoders rotate similarly to potentiometers, but they’re different from potentiometers in that an encoder has full rotation without limits. They output gray code so that you can tell how much and in which direction the encoder has been turned.&lt;/p&gt;
&lt;p&gt;This encoder includes a push-down select switch built into the shaft.&lt;/p&gt;
&lt;p&gt;&lt;a href="http://www.sparkfun.com/datasheets/Components/TW-700198.pdf"&gt;Datasheet&lt;/a&gt;&lt;/p&gt;</description>
<pad name="C" x="7.493" y="0" drill="1.016"/>
<pad name="SW+" x="-6.985" y="2.54" drill="1.016"/>
<pad name="P$3" x="0" y="6.604" drill="2.286"/>
<pad name="P$4" x="0" y="-6.604" drill="2.286"/>
<pad name="SW-" x="-6.985" y="-2.54" drill="1.016"/>
<pad name="B" x="7.493" y="2.54" drill="1.016"/>
<pad name="A" x="7.493" y="-2.54" drill="1.016"/>
<wire x1="-6.35" y1="6.35" x2="-2.54" y2="6.35" width="0.2032" layer="21"/>
<wire x1="2.54" y1="6.35" x2="6.35" y2="6.35" width="0.2032" layer="21"/>
<wire x1="6.35" y1="6.35" x2="6.35" y2="-6.35" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-6.35" x2="2.54" y2="-6.35" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="-6.35" x2="-6.35" y2="-6.35" width="0.2032" layer="21"/>
<wire x1="-6.35" y1="-6.35" x2="-6.35" y2="-3.81" width="0.2032" layer="21"/>
<wire x1="-6.35" y1="6.35" x2="-6.35" y2="3.81" width="0.2032" layer="21"/>
<wire x1="-6.35" y1="1.27" x2="-6.35" y2="-1.27" width="0.2032" layer="21"/>
<text x="5.08" y="6.477" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="5.08" y="-6.477" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="REED_SWITCH">
<description>&lt;h3&gt;Reed Switch&lt;/h3&gt;
&lt;p&gt;A reed switch is a magnetically-actuated switch. When the device is exposed to a magnetic field, the two ferrous materials inside the switch pull together and the switch closes.&lt;/p&gt;</description>
<wire x1="1.905" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0" x2="1.905" y2="1.27" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<circle x="2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<text x="0" y="2.032" size="1.778" layer="95" font="vector" align="bottom-center">&gt;NAME</text>
<text x="0" y="-2.032" size="1.778" layer="96" font="vector" align="top-center">&gt;VALUE</text>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="2"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<wire x1="-4.064" y1="0" x2="-2.286" y2="1.778" width="0.254" layer="94" curve="-90"/>
<wire x1="-2.286" y1="1.778" x2="2.286" y2="1.778" width="0.254" layer="94"/>
<wire x1="2.286" y1="1.778" x2="4.064" y2="0" width="0.254" layer="94" curve="-90"/>
<wire x1="4.064" y1="0" x2="2.286" y2="-1.778" width="0.254" layer="94" curve="-90"/>
<wire x1="-2.286" y1="-1.778" x2="-4.064" y2="0" width="0.254" layer="94" curve="-90"/>
<wire x1="2.286" y1="-1.778" x2="-2.286" y2="-1.778" width="0.254" layer="94"/>
</symbol>
<symbol name="SWITCH-MOMENTARY-2">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;</description>
<wire x1="1.905" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="1.905" y2="1.27" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<circle x="2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<text x="0" y="1.524" size="1.778" layer="95" font="vector" align="bottom-center">&gt;NAME</text>
<text x="0" y="-0.508" size="1.778" layer="96" font="vector" align="top-center">&gt;VALUE</text>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="2"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="ROT_ENCODER">
<description>&lt;h3&gt;Rotary Encoder w/ Select Switch&lt;/h3&gt;
&lt;p&gt;Encoders rotate similarly to potentiometers, but they’re different from potentiometers in that an encoder has full rotation without limits. They output gray code so that you can tell how much and in which direction the encoder has been turned.&lt;/p&gt;</description>
<pin name="SW+" x="-10.16" y="5.08" visible="off" length="middle"/>
<pin name="SW-" x="-10.16" y="-5.08" visible="off" length="middle"/>
<pin name="B" x="10.16" y="5.08" visible="pin" length="middle" rot="R180"/>
<pin name="C" x="10.16" y="0" visible="pin" length="middle" rot="R180"/>
<pin name="A" x="10.16" y="-5.08" visible="pin" length="middle" rot="R180"/>
<wire x1="-5.08" y1="5.08" x2="-2.54" y2="5.08" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="2.54" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.524" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="-5.08" y1="-5.08" x2="-2.54" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-2.54" y2="-1.778" width="0.1524" layer="94"/>
<wire x1="-7.62" y1="7.62" x2="7.62" y2="7.62" width="0.254" layer="94"/>
<wire x1="7.62" y1="7.62" x2="7.62" y2="-7.62" width="0.254" layer="94"/>
<wire x1="7.62" y1="-7.62" x2="-7.62" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-7.62" x2="-7.62" y2="7.62" width="0.254" layer="94"/>
<text x="-7.62" y="7.874" size="1.778" layer="95" font="vector">&gt;NAME</text>
<text x="-7.62" y="-7.874" size="1.778" layer="96" font="vector" align="top-left">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="REED_SWITCH" prefix="S">
<description>&lt;h3&gt;Reed Switch&lt;/h3&gt;
&lt;p&gt;A reed switch is a magnetically-actuated switch. When the device is exposed to a magnetic field, the two ferrous materials inside the switch pull together and the switch closes.&lt;/p&gt;
&lt;h4&gt;Variant Overview&lt;/h4&gt;
&lt;h5&gt;PTH-GLASS&lt;/h5&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/8642"&gt;Reed Switch&lt;/a&gt; (COM-08642) - PTH reed switch w/ glass body (6.0 x 0.9 mm)&lt;/li&gt;
&lt;/ul&gt;
&lt;h5&gt;PTH-INSULATED&lt;/h5&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/10601"&gt;Reed Switch - Insulated&lt;/a&gt; (COM-10601) - PTH reed switch w/ insulated body (15.1 x 4.2 mm)&lt;/li&gt;
&lt;/ul&gt;</description>
<gates>
<gate name="G$1" symbol="REED_SWITCH" x="0" y="0"/>
</gates>
<devices>
<device name="-PTH-GLASS" package="REED_SWITCH_GLASS">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-09282"/>
<attribute name="SF_SKU" value="COM-08642"/>
</technology>
</technologies>
</device>
<device name="-PTH-INSULATED" package="REED_SWITCH_PLASTIC">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-10467"/>
<attribute name="SF_SKU" value="COM-10601"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MOMENTARY-SWITCH-SPST" prefix="S">
<description>&lt;h3&gt;Momentary Switch (Pushbutton) - SPST&lt;/h3&gt;
&lt;p&gt;Normally-open (NO) SPST momentary switches (buttons, pushbuttons).&lt;/p&gt;
&lt;h4&gt;Variants&lt;/h4&gt;
&lt;h5&gt;PTH-12MM - 12mm square, through-hole&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/9190"&gt;Momentary Pushbutton Switch - 12mm Square&lt;/a&gt; (COM-09190)&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;PTH-6.0MM, PTH-6.0MM-KIT - 6.0mm square, through-hole&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/97"&gt;Mini Pushbutton Switch&lt;/a&gt; (COM-00097)&lt;/li&gt;
&lt;li&gt;KIT package intended for soldering kit's - only one side of pads' copper is exposed.&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;PTH-RIGHT-ANGLE-KIT - Right-angle, through-hole&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/10791"&gt;Right Angle Tactile Button&lt;/a&gt; - Used on &lt;a href="https://www.sparkfun.com/products/11734"&gt;
SparkFun BigTime Watch Kit&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-12MM - 12mm square, surface-mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/12993"&gt;Tactile Button - SMD (12mm)&lt;/a&gt; (COM-12993)&lt;/li&gt;
&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/11888"&gt;SparkFun PicoBoard&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-4.5MM - 4.5mm Square Trackball Switch&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/13169"&gt;SparkFun Blackberry Trackballer Breakout&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-4.6MMX2.8MM -  4.60mm x 2.80mm, surface mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/13664"&gt;SparkFun SAMD21 Mini Breakout&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-5.2MM, SMD-5.2-REDUNDANT - 5.2mm square, surface-mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/8720"&gt;Mini Pushbutton Switch - SMD&lt;/a&gt; (COM-08720)&lt;/li&gt;
&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/11114"&gt;Arduino Pro Mini&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;REDUNDANT package connects both switch circuits together&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-6.0X3.5MM - 6.0 x 3.5mm, surface mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/8229"&gt;Momentary Reset Switch SMD&lt;/a&gt; (COM-08229)&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-6.2MM-TALL - 6.2mm square, surface mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/12992"&gt;Tactile Button - SMD (6mm)&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/12651"&gt;SparkFun Digital Sandbox&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;
&lt;h5&gt;SMD-RIGHT-ANGLE - Right-angle, surface mount&lt;/h5&gt;
&lt;ul&gt;&lt;li&gt;Used on &lt;a href="https://www.sparkfun.com/products/13036"&gt;SparkFun Block for Intel® Edison - Arduino&lt;/a&gt;&lt;/li&gt;&lt;/ul&gt;</description>
<gates>
<gate name="G$1" symbol="SWITCH-MOMENTARY-2" x="0" y="0"/>
</gates>
<devices>
<device name="-PTH-6.0MM" package="TACTILE_SWITCH_PTH_6.0MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value=" SWCH-08441"/>
<attribute name="SF_SKU" value="COM-00097"/>
</technology>
</technologies>
</device>
<device name="-SMD-4.5MM" package="TACTILE_SWITCH_SMD_4.5MM">
<connects>
<connect gate="G$1" pin="1" pad="2 3"/>
<connect gate="G$1" pin="2" pad="1 4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-09213"/>
</technology>
</technologies>
</device>
<device name="-PTH-12MM" package="TACTILE_SWITCH_PTH_12MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-09185"/>
<attribute name="SF_SKU" value="COM-09190"/>
</technology>
</technologies>
</device>
<device name="-SMD-6.0X3.5MM" package="TACTILE_SWITCH_SMD_6.0X3.5MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-00815"/>
<attribute name="SF_SKU" value="COM-08229"/>
</technology>
</technologies>
</device>
<device name="-SMD-6.2MM-TALL" package="TACTILE_SWITCH_SMD_6.2MM_TALL">
<connects>
<connect gate="G$1" pin="1" pad="A2"/>
<connect gate="G$1" pin="2" pad="B2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-11966"/>
<attribute name="SF_SKU" value="COM-12992"/>
</technology>
</technologies>
</device>
<device name="-PTH-RIGHT-ANGLE-KIT" package="TACTILE_SWITCH_PTH_RIGHT_ANGLE_KIT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-10672"/>
<attribute name="SF_SKU" value="COM-10791"/>
</technology>
</technologies>
</device>
<device name="-SMD-12MM" package="TACTILE_SWITCH_SMD_12MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-11967"/>
<attribute name="SF_SKU" value="COM-12993"/>
</technology>
</technologies>
</device>
<device name="-PTH-6.0MM-KIT" package="TACTILE_SWITCH_PTH_6.0MM_KIT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-08441"/>
<attribute name="SF_SKU" value="COM-00097 "/>
</technology>
</technologies>
</device>
<device name="-SMD-5.2MM" package="TACTILE_SWITCH_SMD_5.2MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-08247"/>
<attribute name="SF_SKU" value="COM-08720"/>
</technology>
</technologies>
</device>
<device name="-SMD-5.2-REDUNDANT" package="TACTILE_SWITCH_SMD_5.2MM">
<connects>
<connect gate="G$1" pin="1" pad="1 2"/>
<connect gate="G$1" pin="2" pad="3 4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-08247"/>
<attribute name="SF_SKU" value="COM-08720"/>
</technology>
</technologies>
</device>
<device name="-SMD-RIGHT-ANGLE" package="TACTILE_SWITCH_SMD_RIGHT_ANGLE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-12265" constant="no"/>
</technology>
</technologies>
</device>
<device name="-SMD-4.6X2.8MM" package="TACTILE_SWITCH_SMD_4.6X2.8MM">
<connects>
<connect gate="G$1" pin="1" pad="1 2"/>
<connect gate="G$1" pin="2" pad="3 4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-13065"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ENCODER-SWITCH" prefix="S">
<description>&lt;h3&gt;Rotary Encoder w/ Select Switch&lt;/h3&gt;
&lt;p&gt;Encoders rotate similarly to potentiometers, but they’re different from potentiometers in that an encoder has full rotation without limits. They output gray code so that you can tell how much and in which direction the encoder has been turned.&lt;/p&gt;
&lt;p&gt;This encoder includes a push-down select switch built into the shaft.&lt;/p&gt;
&lt;h4&gt;SparkFun Products&lt;/h4&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/9117"&gt;Rotary Encoder&lt;/a&gt; (COM-09117)&lt;/li&gt;
&lt;/ul&gt;</description>
<gates>
<gate name="G$1" symbol="ROT_ENCODER" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ROTARY_ENC_PLAIN">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="SW+" pad="SW+"/>
<connect gate="G$1" pin="SW-" pad="SW-"/>
</connects>
<technologies>
<technology name="">
<attribute name="SF_SKU" value="COM-09117"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-LED">
<description>&lt;h3&gt;SparkFun LEDs&lt;/h3&gt;
This library contains discrete LEDs for illumination or indication, but no displays.
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="7-SEGMENT-4DIGIT-20MM">
<wire x1="-32" y1="13" x2="32" y2="13" width="0.254" layer="21"/>
<wire x1="32" y1="13" x2="32" y2="-13" width="0.254" layer="21"/>
<wire x1="32" y1="-13" x2="-32" y2="-13" width="0.254" layer="21"/>
<wire x1="-32" y1="-13" x2="-32" y2="13" width="0.254" layer="21"/>
<pad name="1" x="-6.35" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="2" x="-3.81" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="3" x="-1.27" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="4" x="1.27" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="5" x="3.81" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="6" x="6.35" y="-10.9982" drill="0.9" diameter="1.8796"/>
<pad name="7" x="6.35" y="10.9982" drill="0.9" diameter="1.8796"/>
<pad name="8" x="3.81" y="10.9982" drill="0.9" diameter="1.8796"/>
<pad name="9" x="1.27" y="10.9982" drill="0.9" diameter="1.8796"/>
<pad name="10" x="-1.27" y="10.9982" drill="0.9" diameter="1.8796"/>
<pad name="11" x="-3.81" y="10.9982" drill="0.9" diameter="1.8796"/>
<pad name="12" x="-6.35" y="10.9982" drill="0.9" diameter="1.8796"/>
<wire x1="-25.2" y1="7.8" x2="-23.8" y2="6.5" width="0.254" layer="51"/>
<wire x1="-23.8" y1="6.5" x2="-25" y2="2.4" width="0.254" layer="51"/>
<wire x1="-25" y1="2.4" x2="-27" y2="0.5" width="0.254" layer="51"/>
<wire x1="-27" y1="0.5" x2="-27.9" y2="1.5" width="0.254" layer="51"/>
<wire x1="-27.9" y1="1.5" x2="-26.3" y2="7.2" width="0.254" layer="51"/>
<wire x1="-26.3" y1="7.2" x2="-25.2" y2="7.8" width="0.254" layer="51"/>
<wire x1="-26.7" y1="0.1" x2="-25.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="-25.5" y1="1.2" x2="-19.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="-19.5" y1="1.2" x2="-18.5" y2="0.1" width="0.254" layer="51"/>
<wire x1="-18.5" y1="0.1" x2="-20" y2="-1.2" width="0.254" layer="51"/>
<wire x1="-20" y1="-1.2" x2="-25.6" y2="-1.2" width="0.254" layer="51"/>
<wire x1="-25.6" y1="-1.2" x2="-26.7" y2="0.1" width="0.254" layer="51"/>
<wire x1="-16.1" y1="7.7" x2="-15.2" y2="6.9" width="0.254" layer="51"/>
<wire x1="-15.2" y1="6.9" x2="-16.6" y2="1.2" width="0.254" layer="51"/>
<wire x1="-16.6" y1="1.2" x2="-17.9" y2="0.4" width="0.254" layer="51"/>
<wire x1="-17.9" y1="0.4" x2="-18.9" y2="1.5" width="0.254" layer="51"/>
<wire x1="-18.9" y1="1.5" x2="-17.6" y2="6.3" width="0.254" layer="51"/>
<wire x1="-17.6" y1="6.3" x2="-16.1" y2="7.7" width="0.254" layer="51"/>
<wire x1="-27.4" y1="0" x2="-26.1" y2="-1.6" width="0.254" layer="51"/>
<wire x1="-26.1" y1="-1.6" x2="-27.2" y2="-5.9" width="0.254" layer="51"/>
<wire x1="-27.2" y1="-5.9" x2="-29.3" y2="-7.5" width="0.254" layer="51"/>
<wire x1="-29.3" y1="-7.5" x2="-30.1" y2="-6.6" width="0.254" layer="51"/>
<wire x1="-30.1" y1="-6.6" x2="-28.7" y2="-1.4" width="0.254" layer="51"/>
<wire x1="-28.7" y1="-1.4" x2="-27.4" y2="0" width="0.254" layer="51"/>
<wire x1="-18.3" y1="-0.4" x2="-17.4" y2="-1.4" width="0.254" layer="51"/>
<wire x1="-17.4" y1="-1.4" x2="-18.6" y2="-6.3" width="0.254" layer="51"/>
<wire x1="-18.6" y1="-6.3" x2="-20" y2="-7.7" width="0.254" layer="51"/>
<wire x1="-20" y1="-7.7" x2="-21.2" y2="-6.2" width="0.254" layer="51"/>
<wire x1="-21.2" y1="-6.2" x2="-19.8" y2="-1.8" width="0.254" layer="51"/>
<wire x1="-19.8" y1="-1.8" x2="-18.3" y2="-0.4" width="0.254" layer="51"/>
<wire x1="-24.5" y1="7.9" x2="-23.4" y2="9" width="0.254" layer="51" curve="-3.579821"/>
<wire x1="-23.4" y1="9" x2="-17" y2="9" width="0.254" layer="51"/>
<wire x1="-17" y1="9" x2="-16.4" y2="8.1" width="0.254" layer="51"/>
<wire x1="-16.4" y1="8.1" x2="-17.9" y2="6.7" width="0.254" layer="51"/>
<wire x1="-17.9" y1="6.7" x2="-23.2" y2="6.7" width="0.254" layer="51"/>
<wire x1="-23.2" y1="6.7" x2="-24.5" y2="7.9" width="0.254" layer="51"/>
<wire x1="-28.7" y1="-7.8" x2="-26.9" y2="-6.4" width="0.254" layer="51"/>
<wire x1="-26.9" y1="-6.4" x2="-21.7" y2="-6.4" width="0.254" layer="51"/>
<wire x1="-21.7" y1="-6.4" x2="-20.4" y2="-8.1" width="0.254" layer="51"/>
<wire x1="-20.4" y1="-8.1" x2="-21.1" y2="-8.8" width="0.254" layer="51"/>
<wire x1="-21.1" y1="-8.8" x2="-28.1" y2="-8.8" width="0.254" layer="51"/>
<wire x1="-28.1" y1="-8.8" x2="-28.7" y2="-7.8" width="0.254" layer="51"/>
<circle x="-16.8" y="-7.6" radius="1.17046875" width="0.254" layer="51"/>
<wire x1="-10.2" y1="7.8" x2="-8.8" y2="6.5" width="0.254" layer="51"/>
<wire x1="-8.8" y1="6.5" x2="-10" y2="2.4" width="0.254" layer="51"/>
<wire x1="-10" y1="2.4" x2="-12" y2="0.5" width="0.254" layer="51"/>
<wire x1="-12" y1="0.5" x2="-12.9" y2="1.5" width="0.254" layer="51"/>
<wire x1="-12.9" y1="1.5" x2="-11.3" y2="7.2" width="0.254" layer="51"/>
<wire x1="-11.3" y1="7.2" x2="-10.2" y2="7.8" width="0.254" layer="51"/>
<wire x1="-11.7" y1="0.1" x2="-10.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="-10.5" y1="1.2" x2="-4.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="-4.5" y1="1.2" x2="-3.5" y2="0.1" width="0.254" layer="51"/>
<wire x1="-3.5" y1="0.1" x2="-5" y2="-1.2" width="0.254" layer="51"/>
<wire x1="-5" y1="-1.2" x2="-10.6" y2="-1.2" width="0.254" layer="51"/>
<wire x1="-10.6" y1="-1.2" x2="-11.7" y2="0.1" width="0.254" layer="51"/>
<wire x1="-1.1" y1="7.7" x2="-0.2" y2="6.9" width="0.254" layer="51"/>
<wire x1="-0.2" y1="6.9" x2="-1.6" y2="1.2" width="0.254" layer="51"/>
<wire x1="-1.6" y1="1.2" x2="-2.9" y2="0.4" width="0.254" layer="51"/>
<wire x1="-2.9" y1="0.4" x2="-3.9" y2="1.5" width="0.254" layer="51"/>
<wire x1="-3.9" y1="1.5" x2="-2.6" y2="6.3" width="0.254" layer="51"/>
<wire x1="-2.6" y1="6.3" x2="-1.1" y2="7.7" width="0.254" layer="51"/>
<wire x1="-12.4" y1="0" x2="-11.1" y2="-1.6" width="0.254" layer="51"/>
<wire x1="-11.1" y1="-1.6" x2="-12.2" y2="-5.9" width="0.254" layer="51"/>
<wire x1="-12.2" y1="-5.9" x2="-14.3" y2="-7.5" width="0.254" layer="51"/>
<wire x1="-14.3" y1="-7.5" x2="-15.1" y2="-6.6" width="0.254" layer="51"/>
<wire x1="-15.1" y1="-6.6" x2="-13.7" y2="-1.4" width="0.254" layer="51"/>
<wire x1="-13.7" y1="-1.4" x2="-12.4" y2="0" width="0.254" layer="51"/>
<wire x1="-3.3" y1="-0.4" x2="-2.4" y2="-1.4" width="0.254" layer="51"/>
<wire x1="-2.4" y1="-1.4" x2="-3.6" y2="-6.3" width="0.254" layer="51"/>
<wire x1="-3.6" y1="-6.3" x2="-5" y2="-7.7" width="0.254" layer="51"/>
<wire x1="-5" y1="-7.7" x2="-6.2" y2="-6.2" width="0.254" layer="51"/>
<wire x1="-6.2" y1="-6.2" x2="-4.8" y2="-1.8" width="0.254" layer="51"/>
<wire x1="-4.8" y1="-1.8" x2="-3.3" y2="-0.4" width="0.254" layer="51"/>
<wire x1="-9.5" y1="7.9" x2="-8.4" y2="9" width="0.254" layer="51" curve="-3.579821"/>
<wire x1="-8.4" y1="9" x2="-2" y2="9" width="0.254" layer="51"/>
<wire x1="-2" y1="9" x2="-1.4" y2="8.1" width="0.254" layer="51"/>
<wire x1="-1.4" y1="8.1" x2="-2.9" y2="6.7" width="0.254" layer="51"/>
<wire x1="-2.9" y1="6.7" x2="-8.2" y2="6.7" width="0.254" layer="51"/>
<wire x1="-8.2" y1="6.7" x2="-9.5" y2="7.9" width="0.254" layer="51"/>
<wire x1="-13.7" y1="-7.8" x2="-11.9" y2="-6.4" width="0.254" layer="51"/>
<wire x1="-11.9" y1="-6.4" x2="-6.7" y2="-6.4" width="0.254" layer="51"/>
<wire x1="-6.7" y1="-6.4" x2="-5.4" y2="-8.1" width="0.254" layer="51"/>
<wire x1="-5.4" y1="-8.1" x2="-6.1" y2="-8.8" width="0.254" layer="51"/>
<wire x1="-6.1" y1="-8.8" x2="-13.1" y2="-8.8" width="0.254" layer="51"/>
<wire x1="-13.1" y1="-8.8" x2="-13.7" y2="-7.8" width="0.254" layer="51"/>
<circle x="-1.8" y="-7.6" radius="1.17046875" width="0.254" layer="51"/>
<wire x1="4.8" y1="7.8" x2="6.2" y2="6.5" width="0.254" layer="51"/>
<wire x1="6.2" y1="6.5" x2="5" y2="2.4" width="0.254" layer="51"/>
<wire x1="5" y1="2.4" x2="3" y2="0.5" width="0.254" layer="51"/>
<wire x1="3" y1="0.5" x2="2.1" y2="1.5" width="0.254" layer="51"/>
<wire x1="2.1" y1="1.5" x2="3.7" y2="7.2" width="0.254" layer="51"/>
<wire x1="3.7" y1="7.2" x2="4.8" y2="7.8" width="0.254" layer="51"/>
<wire x1="3.3" y1="0.1" x2="4.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="4.5" y1="1.2" x2="10.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="10.5" y1="1.2" x2="11.5" y2="0.1" width="0.254" layer="51"/>
<wire x1="11.5" y1="0.1" x2="10" y2="-1.2" width="0.254" layer="51"/>
<wire x1="10" y1="-1.2" x2="4.4" y2="-1.2" width="0.254" layer="51"/>
<wire x1="4.4" y1="-1.2" x2="3.3" y2="0.1" width="0.254" layer="51"/>
<wire x1="13.9" y1="7.7" x2="14.8" y2="6.9" width="0.254" layer="51"/>
<wire x1="14.8" y1="6.9" x2="13.4" y2="1.2" width="0.254" layer="51"/>
<wire x1="13.4" y1="1.2" x2="12.1" y2="0.4" width="0.254" layer="51"/>
<wire x1="12.1" y1="0.4" x2="11.1" y2="1.5" width="0.254" layer="51"/>
<wire x1="11.1" y1="1.5" x2="12.4" y2="6.3" width="0.254" layer="51"/>
<wire x1="12.4" y1="6.3" x2="13.9" y2="7.7" width="0.254" layer="51"/>
<wire x1="2.6" y1="0" x2="3.9" y2="-1.6" width="0.254" layer="51"/>
<wire x1="3.9" y1="-1.6" x2="2.8" y2="-5.9" width="0.254" layer="51"/>
<wire x1="2.8" y1="-5.9" x2="0.7" y2="-7.5" width="0.254" layer="51"/>
<wire x1="0.7" y1="-7.5" x2="-0.1" y2="-6.6" width="0.254" layer="51"/>
<wire x1="-0.1" y1="-6.6" x2="1.3" y2="-1.4" width="0.254" layer="51"/>
<wire x1="1.3" y1="-1.4" x2="2.6" y2="0" width="0.254" layer="51"/>
<wire x1="11.7" y1="-0.4" x2="12.6" y2="-1.4" width="0.254" layer="51"/>
<wire x1="12.6" y1="-1.4" x2="11.4" y2="-6.3" width="0.254" layer="51"/>
<wire x1="11.4" y1="-6.3" x2="10" y2="-7.7" width="0.254" layer="51"/>
<wire x1="10" y1="-7.7" x2="8.8" y2="-6.2" width="0.254" layer="51"/>
<wire x1="8.8" y1="-6.2" x2="10.2" y2="-1.8" width="0.254" layer="51"/>
<wire x1="10.2" y1="-1.8" x2="11.7" y2="-0.4" width="0.254" layer="51"/>
<wire x1="5.5" y1="7.9" x2="6.6" y2="9" width="0.254" layer="51" curve="-3.579821"/>
<wire x1="6.6" y1="9" x2="13" y2="9" width="0.254" layer="51"/>
<wire x1="13" y1="9" x2="13.6" y2="8.1" width="0.254" layer="51"/>
<wire x1="13.6" y1="8.1" x2="12.1" y2="6.7" width="0.254" layer="51"/>
<wire x1="12.1" y1="6.7" x2="6.8" y2="6.7" width="0.254" layer="51"/>
<wire x1="6.8" y1="6.7" x2="5.5" y2="7.9" width="0.254" layer="51"/>
<wire x1="1.3" y1="-7.8" x2="3.1" y2="-6.4" width="0.254" layer="51"/>
<wire x1="3.1" y1="-6.4" x2="8.3" y2="-6.4" width="0.254" layer="51"/>
<wire x1="8.3" y1="-6.4" x2="9.6" y2="-8.1" width="0.254" layer="51"/>
<wire x1="9.6" y1="-8.1" x2="8.9" y2="-8.8" width="0.254" layer="51"/>
<wire x1="8.9" y1="-8.8" x2="1.9" y2="-8.8" width="0.254" layer="51"/>
<wire x1="1.9" y1="-8.8" x2="1.3" y2="-7.8" width="0.254" layer="51"/>
<circle x="13.2" y="-7.6" radius="1.17046875" width="0.254" layer="51"/>
<wire x1="19.8" y1="7.8" x2="21.2" y2="6.5" width="0.254" layer="51"/>
<wire x1="21.2" y1="6.5" x2="20" y2="2.4" width="0.254" layer="51"/>
<wire x1="20" y1="2.4" x2="18" y2="0.5" width="0.254" layer="51"/>
<wire x1="18" y1="0.5" x2="17.1" y2="1.5" width="0.254" layer="51"/>
<wire x1="17.1" y1="1.5" x2="18.7" y2="7.2" width="0.254" layer="51"/>
<wire x1="18.7" y1="7.2" x2="19.8" y2="7.8" width="0.254" layer="51"/>
<wire x1="18.3" y1="0.1" x2="19.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="19.5" y1="1.2" x2="25.5" y2="1.2" width="0.254" layer="51"/>
<wire x1="25.5" y1="1.2" x2="26.5" y2="0.1" width="0.254" layer="51"/>
<wire x1="26.5" y1="0.1" x2="25" y2="-1.2" width="0.254" layer="51"/>
<wire x1="25" y1="-1.2" x2="19.4" y2="-1.2" width="0.254" layer="51"/>
<wire x1="19.4" y1="-1.2" x2="18.3" y2="0.1" width="0.254" layer="51"/>
<wire x1="28.9" y1="7.7" x2="29.8" y2="6.9" width="0.254" layer="51"/>
<wire x1="29.8" y1="6.9" x2="28.4" y2="1.2" width="0.254" layer="51"/>
<wire x1="28.4" y1="1.2" x2="27.1" y2="0.4" width="0.254" layer="51"/>
<wire x1="27.1" y1="0.4" x2="26.1" y2="1.5" width="0.254" layer="51"/>
<wire x1="26.1" y1="1.5" x2="27.4" y2="6.3" width="0.254" layer="51"/>
<wire x1="27.4" y1="6.3" x2="28.9" y2="7.7" width="0.254" layer="51"/>
<wire x1="17.6" y1="0" x2="18.9" y2="-1.6" width="0.254" layer="51"/>
<wire x1="18.9" y1="-1.6" x2="17.8" y2="-5.9" width="0.254" layer="51"/>
<wire x1="17.8" y1="-5.9" x2="15.7" y2="-7.5" width="0.254" layer="51"/>
<wire x1="15.7" y1="-7.5" x2="14.9" y2="-6.6" width="0.254" layer="51"/>
<wire x1="14.9" y1="-6.6" x2="16.3" y2="-1.4" width="0.254" layer="51"/>
<wire x1="16.3" y1="-1.4" x2="17.6" y2="0" width="0.254" layer="51"/>
<wire x1="26.7" y1="-0.4" x2="27.6" y2="-1.4" width="0.254" layer="51"/>
<wire x1="27.6" y1="-1.4" x2="26.4" y2="-6.3" width="0.254" layer="51"/>
<wire x1="26.4" y1="-6.3" x2="25" y2="-7.7" width="0.254" layer="51"/>
<wire x1="25" y1="-7.7" x2="23.8" y2="-6.2" width="0.254" layer="51"/>
<wire x1="23.8" y1="-6.2" x2="25.2" y2="-1.8" width="0.254" layer="51"/>
<wire x1="25.2" y1="-1.8" x2="26.7" y2="-0.4" width="0.254" layer="51"/>
<wire x1="20.5" y1="7.9" x2="21.6" y2="9" width="0.254" layer="51" curve="-3.579821"/>
<wire x1="21.6" y1="9" x2="28" y2="9" width="0.254" layer="51"/>
<wire x1="28" y1="9" x2="28.6" y2="8.1" width="0.254" layer="51"/>
<wire x1="28.6" y1="8.1" x2="27.1" y2="6.7" width="0.254" layer="51"/>
<wire x1="27.1" y1="6.7" x2="21.8" y2="6.7" width="0.254" layer="51"/>
<wire x1="21.8" y1="6.7" x2="20.5" y2="7.9" width="0.254" layer="51"/>
<wire x1="16.3" y1="-7.8" x2="18.1" y2="-6.4" width="0.254" layer="51"/>
<wire x1="18.1" y1="-6.4" x2="23.3" y2="-6.4" width="0.254" layer="51"/>
<wire x1="23.3" y1="-6.4" x2="24.6" y2="-8.1" width="0.254" layer="51"/>
<wire x1="24.6" y1="-8.1" x2="23.9" y2="-8.8" width="0.254" layer="51"/>
<wire x1="23.9" y1="-8.8" x2="16.9" y2="-8.8" width="0.254" layer="51"/>
<wire x1="16.9" y1="-8.8" x2="16.3" y2="-7.8" width="0.254" layer="51"/>
<circle x="28.2" y="-7.6" radius="1.17046875" width="0.254" layer="51"/>
<polygon width="0.254" layer="21">
<vertex x="-30.1" y="-6.6"/>
<vertex x="-28.7" y="-1.4"/>
<vertex x="-27.2" y="0"/>
<vertex x="-26.1" y="-1.6"/>
<vertex x="-27.2" y="-5.8"/>
<vertex x="-29.4" y="-7.4"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-28.8" y="-8"/>
<vertex x="-26.7" y="-6.4"/>
<vertex x="-21.8" y="-6.4"/>
<vertex x="-20.4" y="-8.1"/>
<vertex x="-21.1" y="-8.8"/>
<vertex x="-28.1" y="-8.8"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-21.2" y="-5.9"/>
<vertex x="-19.8" y="-1.8"/>
<vertex x="-18.1" y="-0.3"/>
<vertex x="-17.4" y="-1.4"/>
<vertex x="-18.6" y="-6.3"/>
<vertex x="-19.9" y="-7.5"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-20.2" y="-1.2"/>
<vertex x="-25.5" y="-1.2"/>
<vertex x="-26.5" y="0.1"/>
<vertex x="-25.2" y="1.2"/>
<vertex x="-19.7" y="1.2"/>
<vertex x="-18.7" y="0.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-18" y="0.3"/>
<vertex x="-18.9" y="1.5"/>
<vertex x="-17.6" y="6.1"/>
<vertex x="-16" y="7.6"/>
<vertex x="-15.2" y="6.9"/>
<vertex x="-16.6" y="1.2"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-18" y="6.7"/>
<vertex x="-23.3" y="6.7"/>
<vertex x="-24.7" y="8.1"/>
<vertex x="-23.7" y="9"/>
<vertex x="-17" y="9"/>
<vertex x="-16.4" y="8.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-23.9" y="6.3"/>
<vertex x="-25.3" y="7.7"/>
<vertex x="-26.3" y="7.2"/>
<vertex x="-27.9" y="1.5"/>
<vertex x="-27.1" y="0.5"/>
<vertex x="-25" y="2.4"/>
</polygon>
<circle x="-16.8" y="-7.6" radius="0.412309375" width="1.5" layer="21"/>
<polygon width="0.254" layer="21">
<vertex x="-15.1" y="-6.6"/>
<vertex x="-13.7" y="-1.4"/>
<vertex x="-12.2" y="0"/>
<vertex x="-11.1" y="-1.6"/>
<vertex x="-12.2" y="-5.8"/>
<vertex x="-14.4" y="-7.4"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-13.8" y="-8"/>
<vertex x="-11.7" y="-6.4"/>
<vertex x="-6.8" y="-6.4"/>
<vertex x="-5.4" y="-8.1"/>
<vertex x="-6.1" y="-8.8"/>
<vertex x="-13.1" y="-8.8"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-6.2" y="-5.9"/>
<vertex x="-4.8" y="-1.8"/>
<vertex x="-3.1" y="-0.3"/>
<vertex x="-2.4" y="-1.4"/>
<vertex x="-3.6" y="-6.3"/>
<vertex x="-4.9" y="-7.5"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-5.2" y="-1.2"/>
<vertex x="-10.5" y="-1.2"/>
<vertex x="-11.5" y="0.1"/>
<vertex x="-10.2" y="1.2"/>
<vertex x="-4.7" y="1.2"/>
<vertex x="-3.7" y="0.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-3" y="0.3"/>
<vertex x="-3.9" y="1.5"/>
<vertex x="-2.6" y="6.1"/>
<vertex x="-1" y="7.6"/>
<vertex x="-0.2" y="6.9"/>
<vertex x="-1.6" y="1.2"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-3" y="6.7"/>
<vertex x="-8.3" y="6.7"/>
<vertex x="-9.7" y="8.1"/>
<vertex x="-8.7" y="9"/>
<vertex x="-2" y="9"/>
<vertex x="-1.4" y="8.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="-8.9" y="6.3"/>
<vertex x="-10.3" y="7.7"/>
<vertex x="-11.3" y="7.2"/>
<vertex x="-12.9" y="1.5"/>
<vertex x="-12.1" y="0.5"/>
<vertex x="-10" y="2.4"/>
</polygon>
<circle x="-1.8" y="-7.6" radius="0.412309375" width="1.5" layer="21"/>
<polygon width="0.254" layer="21">
<vertex x="-0.1" y="-6.6"/>
<vertex x="1.3" y="-1.4"/>
<vertex x="2.8" y="0"/>
<vertex x="3.9" y="-1.6"/>
<vertex x="2.8" y="-5.8"/>
<vertex x="0.6" y="-7.4"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="1.2" y="-8"/>
<vertex x="3.3" y="-6.4"/>
<vertex x="8.2" y="-6.4"/>
<vertex x="9.6" y="-8.1"/>
<vertex x="8.9" y="-8.8"/>
<vertex x="1.9" y="-8.8"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="8.8" y="-5.9"/>
<vertex x="10.2" y="-1.8"/>
<vertex x="11.9" y="-0.3"/>
<vertex x="12.6" y="-1.4"/>
<vertex x="11.4" y="-6.3"/>
<vertex x="10.1" y="-7.5"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="9.8" y="-1.2"/>
<vertex x="4.5" y="-1.2"/>
<vertex x="3.5" y="0.1"/>
<vertex x="4.8" y="1.2"/>
<vertex x="10.3" y="1.2"/>
<vertex x="11.3" y="0.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="12" y="0.3"/>
<vertex x="11.1" y="1.5"/>
<vertex x="12.4" y="6.1"/>
<vertex x="14" y="7.6"/>
<vertex x="14.8" y="6.9"/>
<vertex x="13.4" y="1.2"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="12" y="6.7"/>
<vertex x="6.7" y="6.7"/>
<vertex x="5.3" y="8.1"/>
<vertex x="6.3" y="9"/>
<vertex x="13" y="9"/>
<vertex x="13.6" y="8.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="6.1" y="6.3"/>
<vertex x="4.7" y="7.7"/>
<vertex x="3.7" y="7.2"/>
<vertex x="2.1" y="1.5"/>
<vertex x="2.9" y="0.5"/>
<vertex x="5" y="2.4"/>
</polygon>
<circle x="13.2" y="-7.6" radius="0.412309375" width="1.5" layer="21"/>
<polygon width="0.254" layer="21">
<vertex x="14.9" y="-6.6"/>
<vertex x="16.3" y="-1.4"/>
<vertex x="17.8" y="0"/>
<vertex x="18.9" y="-1.6"/>
<vertex x="17.8" y="-5.8"/>
<vertex x="15.6" y="-7.4"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="16.2" y="-8"/>
<vertex x="18.3" y="-6.4"/>
<vertex x="23.2" y="-6.4"/>
<vertex x="24.6" y="-8.1"/>
<vertex x="23.9" y="-8.8"/>
<vertex x="16.9" y="-8.8"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="23.8" y="-5.9"/>
<vertex x="25.2" y="-1.8"/>
<vertex x="26.9" y="-0.3"/>
<vertex x="27.6" y="-1.4"/>
<vertex x="26.4" y="-6.3"/>
<vertex x="25.1" y="-7.5"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="24.8" y="-1.2"/>
<vertex x="19.5" y="-1.2"/>
<vertex x="18.5" y="0.1"/>
<vertex x="19.8" y="1.2"/>
<vertex x="25.3" y="1.2"/>
<vertex x="26.3" y="0.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="27" y="0.3"/>
<vertex x="26.1" y="1.5"/>
<vertex x="27.4" y="6.1"/>
<vertex x="29" y="7.6"/>
<vertex x="29.8" y="6.9"/>
<vertex x="28.4" y="1.2"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="27" y="6.7"/>
<vertex x="21.7" y="6.7"/>
<vertex x="20.3" y="8.1"/>
<vertex x="21.3" y="9"/>
<vertex x="28" y="9"/>
<vertex x="28.6" y="8.1"/>
</polygon>
<polygon width="0.254" layer="21">
<vertex x="21.1" y="6.3"/>
<vertex x="19.7" y="7.7"/>
<vertex x="18.7" y="7.2"/>
<vertex x="17.1" y="1.5"/>
<vertex x="17.9" y="0.5"/>
<vertex x="20" y="2.4"/>
</polygon>
<circle x="28.2" y="-7.6" radius="0.412309375" width="1.5" layer="21"/>
</package>
<package name="DIP12-HP-BUBBLE">
<wire x1="7.62" y1="2.54" x2="7.62" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="-7.62" y1="-2.54" x2="-7.62" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-7.62" y1="2.54" x2="7.62" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-7.62" y1="-2.54" x2="7.62" y2="-2.54" width="0.2032" layer="21"/>
<text x="-3.81" y="0.635" size="1.27" layer="25" font="vector" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-1.778" size="1.27" layer="27" font="vector" ratio="10">&gt;VALUE</text>
<text x="-3.81" y="-1.778" size="1.27" layer="27" font="vector" ratio="10">&gt;VALUE</text>
<circle x="-6.35" y="-1.27" radius="0.254" width="0.4064" layer="21"/>
<pad name="1" x="-6.35" y="-3.81" drill="0.8636" diameter="1.905" shape="square"/>
<pad name="2" x="-3.81" y="-3.81" drill="0.8636" diameter="1.905"/>
<pad name="3" x="-1.27" y="-3.81" drill="0.8636" diameter="1.905"/>
<pad name="4" x="1.27" y="-3.81" drill="0.8636" diameter="1.905"/>
<pad name="5" x="3.81" y="-3.81" drill="0.8636" diameter="1.905"/>
<pad name="6" x="6.35" y="-3.81" drill="0.8636" diameter="1.905"/>
<pad name="7" x="6.35" y="3.81" drill="0.8636" diameter="1.905"/>
<pad name="8" x="3.81" y="3.81" drill="0.8636" diameter="1.905"/>
<pad name="9" x="1.27" y="3.81" drill="0.8636" diameter="1.905"/>
<pad name="10" x="-1.27" y="3.81" drill="0.8636" diameter="1.905"/>
<pad name="11" x="-3.81" y="3.81" drill="0.8636" diameter="1.905"/>
<pad name="12" x="-6.35" y="3.81" drill="0.8636" diameter="1.905"/>
</package>
</packages>
<symbols>
<symbol name="7-SEGMENT-4DIGIT-COUNTER">
<wire x1="-35.56" y1="16.51" x2="35.56" y2="16.51" width="0.254" layer="94"/>
<wire x1="35.56" y1="16.51" x2="35.56" y2="-15.24" width="0.254" layer="94"/>
<wire x1="35.56" y1="-15.24" x2="-35.56" y2="-15.24" width="0.254" layer="94"/>
<wire x1="-35.56" y1="-15.24" x2="-35.56" y2="16.51" width="0.254" layer="94"/>
<wire x1="-22.59" y1="15.51" x2="-21.32" y2="14.24" width="0.508" layer="94"/>
<wire x1="-21.32" y1="14.24" x2="-22.59" y2="12.97" width="0.508" layer="94"/>
<wire x1="-22.59" y1="12.97" x2="-29.21" y2="12.97" width="0.508" layer="94"/>
<wire x1="-29.21" y1="12.97" x2="-30.48" y2="14.24" width="0.508" layer="94"/>
<wire x1="-30.48" y1="14.24" x2="-29.21" y2="15.51" width="0.508" layer="94"/>
<wire x1="-29.21" y1="15.51" x2="-22.59" y2="15.51" width="0.508" layer="94"/>
<wire x1="-21.066" y1="12.97" x2="-19.796" y2="11.7" width="0.508" layer="94"/>
<wire x1="-21.066" y1="12.97" x2="-22.336" y2="11.7" width="0.508" layer="94"/>
<wire x1="-22.336" y1="11.7" x2="-23.606" y2="6.08" width="0.508" layer="94"/>
<wire x1="-23.606" y1="6.08" x2="-22.336" y2="4.81" width="0.508" layer="94"/>
<wire x1="-22.336" y1="4.81" x2="-21.066" y2="6.08" width="0.508" layer="94"/>
<wire x1="-21.066" y1="6.08" x2="-19.796" y2="11.7" width="0.508" layer="94"/>
<wire x1="-31.75" y1="3.54" x2="-30.48" y2="4.81" width="0.508" layer="94"/>
<wire x1="-31.75" y1="3.54" x2="-30.48" y2="2.27" width="0.508" layer="94"/>
<wire x1="-30.48" y1="2.27" x2="-23.86" y2="2.27" width="0.508" layer="94"/>
<wire x1="-23.86" y1="2.27" x2="-22.59" y2="3.54" width="0.508" layer="94"/>
<wire x1="-22.59" y1="3.54" x2="-23.86" y2="4.81" width="0.508" layer="94"/>
<wire x1="-23.86" y1="4.81" x2="-30.48" y2="4.81" width="0.508" layer="94"/>
<wire x1="-33.02" y1="-7.16" x2="-31.75" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-33.02" y1="-7.16" x2="-31.75" y2="-8.43" width="0.508" layer="94"/>
<wire x1="-31.75" y1="-8.43" x2="-25.13" y2="-8.43" width="0.508" layer="94"/>
<wire x1="-25.13" y1="-8.43" x2="-23.86" y2="-7.16" width="0.508" layer="94"/>
<wire x1="-23.86" y1="-7.16" x2="-25.13" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-25.13" y1="-5.89" x2="-31.75" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-30.734" y1="12.97" x2="-29.464" y2="11.7" width="0.508" layer="94"/>
<wire x1="-30.734" y1="12.97" x2="-32.004" y2="11.7" width="0.508" layer="94"/>
<wire x1="-32.004" y1="11.7" x2="-33.274" y2="6.08" width="0.508" layer="94"/>
<wire x1="-33.274" y1="6.08" x2="-32.004" y2="4.81" width="0.508" layer="94"/>
<wire x1="-32.004" y1="4.81" x2="-30.734" y2="6.08" width="0.508" layer="94"/>
<wire x1="-30.734" y1="6.08" x2="-29.464" y2="11.7" width="0.508" layer="94"/>
<wire x1="-32.004" y1="2.27" x2="-30.734" y2="1" width="0.508" layer="94"/>
<wire x1="-32.004" y1="2.27" x2="-33.274" y2="1" width="0.508" layer="94"/>
<wire x1="-33.274" y1="1" x2="-34.544" y2="-5.62" width="0.508" layer="94"/>
<wire x1="-34.544" y1="-5.62" x2="-33.274" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-33.274" y1="-5.89" x2="-32.004" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-32.004" y1="-4.62" x2="-30.734" y2="1" width="0.508" layer="94"/>
<wire x1="-22.336" y1="2.27" x2="-21.066" y2="1" width="0.508" layer="94"/>
<wire x1="-22.336" y1="2.27" x2="-23.606" y2="1" width="0.508" layer="94"/>
<wire x1="-23.606" y1="1" x2="-24.876" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-24.876" y1="-4.62" x2="-23.606" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-23.606" y1="-5.89" x2="-22.336" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-22.336" y1="-4.62" x2="-21.066" y2="1" width="0.508" layer="94"/>
<wire x1="-4.81" y1="15.51" x2="-3.54" y2="14.24" width="0.508" layer="94"/>
<wire x1="-3.54" y1="14.24" x2="-4.81" y2="12.97" width="0.508" layer="94"/>
<wire x1="-4.81" y1="12.97" x2="-11.43" y2="12.97" width="0.508" layer="94"/>
<wire x1="-11.43" y1="12.97" x2="-12.7" y2="14.24" width="0.508" layer="94"/>
<wire x1="-12.7" y1="14.24" x2="-11.43" y2="15.51" width="0.508" layer="94"/>
<wire x1="-11.43" y1="15.51" x2="-4.81" y2="15.51" width="0.508" layer="94"/>
<wire x1="-3.286" y1="12.97" x2="-2.016" y2="11.7" width="0.508" layer="94"/>
<wire x1="-3.286" y1="12.97" x2="-4.556" y2="11.7" width="0.508" layer="94"/>
<wire x1="-4.556" y1="11.7" x2="-5.826" y2="6.08" width="0.508" layer="94"/>
<wire x1="-5.826" y1="6.08" x2="-4.556" y2="4.81" width="0.508" layer="94"/>
<wire x1="-4.556" y1="4.81" x2="-3.286" y2="6.08" width="0.508" layer="94"/>
<wire x1="-3.286" y1="6.08" x2="-2.016" y2="11.7" width="0.508" layer="94"/>
<wire x1="-13.97" y1="3.54" x2="-12.7" y2="4.81" width="0.508" layer="94"/>
<wire x1="-13.97" y1="3.54" x2="-12.7" y2="2.27" width="0.508" layer="94"/>
<wire x1="-12.7" y1="2.27" x2="-6.08" y2="2.27" width="0.508" layer="94"/>
<wire x1="-6.08" y1="2.27" x2="-4.81" y2="3.54" width="0.508" layer="94"/>
<wire x1="-4.81" y1="3.54" x2="-6.08" y2="4.81" width="0.508" layer="94"/>
<wire x1="-6.08" y1="4.81" x2="-12.7" y2="4.81" width="0.508" layer="94"/>
<wire x1="-15.24" y1="-7.16" x2="-13.97" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-15.24" y1="-7.16" x2="-13.97" y2="-8.43" width="0.508" layer="94"/>
<wire x1="-13.97" y1="-8.43" x2="-7.35" y2="-8.43" width="0.508" layer="94"/>
<wire x1="-7.35" y1="-8.43" x2="-6.08" y2="-7.16" width="0.508" layer="94"/>
<wire x1="-6.08" y1="-7.16" x2="-7.35" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-7.35" y1="-5.89" x2="-13.97" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-12.954" y1="12.97" x2="-11.684" y2="11.7" width="0.508" layer="94"/>
<wire x1="-12.954" y1="12.97" x2="-14.224" y2="11.7" width="0.508" layer="94"/>
<wire x1="-14.224" y1="11.7" x2="-15.494" y2="6.08" width="0.508" layer="94"/>
<wire x1="-15.494" y1="6.08" x2="-14.224" y2="4.81" width="0.508" layer="94"/>
<wire x1="-14.224" y1="4.81" x2="-12.954" y2="6.08" width="0.508" layer="94"/>
<wire x1="-12.954" y1="6.08" x2="-11.684" y2="11.7" width="0.508" layer="94"/>
<wire x1="-14.224" y1="2.27" x2="-12.954" y2="1" width="0.508" layer="94"/>
<wire x1="-14.224" y1="2.27" x2="-15.494" y2="1" width="0.508" layer="94"/>
<wire x1="-15.494" y1="1" x2="-16.764" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-16.764" y1="-4.62" x2="-15.494" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-15.494" y1="-5.89" x2="-14.224" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-14.224" y1="-4.62" x2="-12.954" y2="1" width="0.508" layer="94"/>
<wire x1="-4.556" y1="2.27" x2="-3.286" y2="1" width="0.508" layer="94"/>
<wire x1="-4.556" y1="2.27" x2="-5.826" y2="1" width="0.508" layer="94"/>
<wire x1="-5.826" y1="1" x2="-7.096" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-7.096" y1="-4.62" x2="-5.826" y2="-5.89" width="0.508" layer="94"/>
<wire x1="-5.826" y1="-5.89" x2="-4.556" y2="-4.62" width="0.508" layer="94"/>
<wire x1="-4.556" y1="-4.62" x2="-3.286" y2="1" width="0.508" layer="94"/>
<wire x1="12.97" y1="15.51" x2="14.24" y2="14.24" width="0.508" layer="94"/>
<wire x1="14.24" y1="14.24" x2="12.97" y2="12.97" width="0.508" layer="94"/>
<wire x1="12.97" y1="12.97" x2="6.35" y2="12.97" width="0.508" layer="94"/>
<wire x1="6.35" y1="12.97" x2="5.08" y2="14.24" width="0.508" layer="94"/>
<wire x1="5.08" y1="14.24" x2="6.35" y2="15.51" width="0.508" layer="94"/>
<wire x1="6.35" y1="15.51" x2="12.97" y2="15.51" width="0.508" layer="94"/>
<wire x1="14.494" y1="12.97" x2="15.764" y2="11.7" width="0.508" layer="94"/>
<wire x1="14.494" y1="12.97" x2="13.224" y2="11.7" width="0.508" layer="94"/>
<wire x1="13.224" y1="11.7" x2="11.954" y2="6.08" width="0.508" layer="94"/>
<wire x1="11.954" y1="6.08" x2="13.224" y2="4.81" width="0.508" layer="94"/>
<wire x1="13.224" y1="4.81" x2="14.494" y2="6.08" width="0.508" layer="94"/>
<wire x1="14.494" y1="6.08" x2="15.764" y2="11.7" width="0.508" layer="94"/>
<wire x1="3.81" y1="3.54" x2="5.08" y2="4.81" width="0.508" layer="94"/>
<wire x1="3.81" y1="3.54" x2="5.08" y2="2.27" width="0.508" layer="94"/>
<wire x1="5.08" y1="2.27" x2="11.7" y2="2.27" width="0.508" layer="94"/>
<wire x1="11.7" y1="2.27" x2="12.97" y2="3.54" width="0.508" layer="94"/>
<wire x1="12.97" y1="3.54" x2="11.7" y2="4.81" width="0.508" layer="94"/>
<wire x1="11.7" y1="4.81" x2="5.08" y2="4.81" width="0.508" layer="94"/>
<wire x1="2.54" y1="-7.16" x2="3.81" y2="-5.89" width="0.508" layer="94"/>
<wire x1="2.54" y1="-7.16" x2="3.81" y2="-8.43" width="0.508" layer="94"/>
<wire x1="3.81" y1="-8.43" x2="10.43" y2="-8.43" width="0.508" layer="94"/>
<wire x1="10.43" y1="-8.43" x2="11.7" y2="-7.16" width="0.508" layer="94"/>
<wire x1="11.7" y1="-7.16" x2="10.43" y2="-5.89" width="0.508" layer="94"/>
<wire x1="10.43" y1="-5.89" x2="3.81" y2="-5.89" width="0.508" layer="94"/>
<wire x1="4.826" y1="12.97" x2="6.096" y2="11.7" width="0.508" layer="94"/>
<wire x1="4.826" y1="12.97" x2="3.556" y2="11.7" width="0.508" layer="94"/>
<wire x1="3.556" y1="11.7" x2="2.286" y2="6.08" width="0.508" layer="94"/>
<wire x1="2.286" y1="6.08" x2="3.556" y2="4.81" width="0.508" layer="94"/>
<wire x1="3.556" y1="4.81" x2="4.826" y2="6.08" width="0.508" layer="94"/>
<wire x1="4.826" y1="6.08" x2="6.096" y2="11.7" width="0.508" layer="94"/>
<wire x1="3.556" y1="2.27" x2="4.826" y2="1" width="0.508" layer="94"/>
<wire x1="3.556" y1="2.27" x2="2.286" y2="1" width="0.508" layer="94"/>
<wire x1="2.286" y1="1" x2="1.016" y2="-4.62" width="0.508" layer="94"/>
<wire x1="1.016" y1="-4.62" x2="2.286" y2="-5.89" width="0.508" layer="94"/>
<wire x1="2.286" y1="-5.89" x2="3.556" y2="-4.62" width="0.508" layer="94"/>
<wire x1="3.556" y1="-4.62" x2="4.826" y2="1" width="0.508" layer="94"/>
<wire x1="13.224" y1="2.27" x2="14.494" y2="1" width="0.508" layer="94"/>
<wire x1="13.224" y1="2.27" x2="11.954" y2="1" width="0.508" layer="94"/>
<wire x1="11.954" y1="1" x2="10.684" y2="-4.62" width="0.508" layer="94"/>
<wire x1="10.684" y1="-4.62" x2="11.954" y2="-5.89" width="0.508" layer="94"/>
<wire x1="11.954" y1="-5.89" x2="13.224" y2="-4.62" width="0.508" layer="94"/>
<wire x1="13.224" y1="-4.62" x2="14.494" y2="1" width="0.508" layer="94"/>
<wire x1="30.75" y1="15.51" x2="32.02" y2="14.24" width="0.508" layer="94"/>
<wire x1="32.02" y1="14.24" x2="30.75" y2="12.97" width="0.508" layer="94"/>
<wire x1="30.75" y1="12.97" x2="24.13" y2="12.97" width="0.508" layer="94"/>
<wire x1="24.13" y1="12.97" x2="22.86" y2="14.24" width="0.508" layer="94"/>
<wire x1="22.86" y1="14.24" x2="24.13" y2="15.51" width="0.508" layer="94"/>
<wire x1="24.13" y1="15.51" x2="30.75" y2="15.51" width="0.508" layer="94"/>
<wire x1="32.274" y1="12.97" x2="33.544" y2="11.7" width="0.508" layer="94"/>
<wire x1="32.274" y1="12.97" x2="31.004" y2="11.7" width="0.508" layer="94"/>
<wire x1="31.004" y1="11.7" x2="29.734" y2="6.08" width="0.508" layer="94"/>
<wire x1="29.734" y1="6.08" x2="31.004" y2="4.81" width="0.508" layer="94"/>
<wire x1="31.004" y1="4.81" x2="32.274" y2="6.08" width="0.508" layer="94"/>
<wire x1="32.274" y1="6.08" x2="33.544" y2="11.7" width="0.508" layer="94"/>
<wire x1="21.59" y1="3.54" x2="22.86" y2="4.81" width="0.508" layer="94"/>
<wire x1="21.59" y1="3.54" x2="22.86" y2="2.27" width="0.508" layer="94"/>
<wire x1="22.86" y1="2.27" x2="29.48" y2="2.27" width="0.508" layer="94"/>
<wire x1="29.48" y1="2.27" x2="30.75" y2="3.54" width="0.508" layer="94"/>
<wire x1="30.75" y1="3.54" x2="29.48" y2="4.81" width="0.508" layer="94"/>
<wire x1="29.48" y1="4.81" x2="22.86" y2="4.81" width="0.508" layer="94"/>
<wire x1="20.32" y1="-7.16" x2="21.59" y2="-5.89" width="0.508" layer="94"/>
<wire x1="20.32" y1="-7.16" x2="21.59" y2="-8.43" width="0.508" layer="94"/>
<wire x1="21.59" y1="-8.43" x2="28.21" y2="-8.43" width="0.508" layer="94"/>
<wire x1="28.21" y1="-8.43" x2="29.48" y2="-7.16" width="0.508" layer="94"/>
<wire x1="29.48" y1="-7.16" x2="28.21" y2="-5.89" width="0.508" layer="94"/>
<wire x1="28.21" y1="-5.89" x2="21.59" y2="-5.89" width="0.508" layer="94"/>
<wire x1="22.606" y1="12.97" x2="23.876" y2="11.7" width="0.508" layer="94"/>
<wire x1="22.606" y1="12.97" x2="21.336" y2="11.7" width="0.508" layer="94"/>
<wire x1="21.336" y1="11.7" x2="20.066" y2="6.08" width="0.508" layer="94"/>
<wire x1="20.066" y1="6.08" x2="21.336" y2="4.81" width="0.508" layer="94"/>
<wire x1="21.336" y1="4.81" x2="22.606" y2="6.08" width="0.508" layer="94"/>
<wire x1="22.606" y1="6.08" x2="23.876" y2="11.7" width="0.508" layer="94"/>
<wire x1="21.336" y1="2.27" x2="22.606" y2="1" width="0.508" layer="94"/>
<wire x1="21.336" y1="2.27" x2="20.066" y2="1" width="0.508" layer="94"/>
<wire x1="20.066" y1="1" x2="18.796" y2="-4.62" width="0.508" layer="94"/>
<wire x1="18.796" y1="-4.62" x2="20.066" y2="-5.89" width="0.508" layer="94"/>
<wire x1="20.066" y1="-5.89" x2="21.336" y2="-4.62" width="0.508" layer="94"/>
<wire x1="21.336" y1="-4.62" x2="22.606" y2="1" width="0.508" layer="94"/>
<wire x1="31.004" y1="2.27" x2="32.274" y2="1" width="0.508" layer="94"/>
<wire x1="31.004" y1="2.27" x2="29.734" y2="1" width="0.508" layer="94"/>
<wire x1="29.734" y1="1" x2="28.464" y2="-4.62" width="0.508" layer="94"/>
<wire x1="28.464" y1="-4.62" x2="29.734" y2="-5.89" width="0.508" layer="94"/>
<wire x1="29.734" y1="-5.89" x2="31.004" y2="-4.62" width="0.508" layer="94"/>
<wire x1="31.004" y1="-4.62" x2="32.274" y2="1" width="0.508" layer="94"/>
<circle x="-20.05" y="-7.16" radius="1.27" width="0.508" layer="94"/>
<circle x="-2.27" y="-7.16" radius="1.27" width="0.508" layer="94"/>
<circle x="15.51" y="-7.16" radius="1.27" width="0.508" layer="94"/>
<circle x="33.29" y="-7.16" radius="1.27" width="0.508" layer="94"/>
<text x="-26.654" y="13.732" size="1.016" layer="94" ratio="15">A</text>
<text x="-22.082" y="7.382" size="1.016" layer="94" ratio="15">B</text>
<text x="-23.352" y="-1.318" size="1.016" layer="94" ratio="15">C</text>
<text x="-28.194" y="-7.668" size="1.016" layer="94" ratio="15">D</text>
<text x="-33.02" y="-1.318" size="1.016" layer="94" ratio="15">E</text>
<text x="-31.75" y="7.382" size="1.016" layer="94" ratio="15">F</text>
<text x="-26.924" y="3.032" size="1.016" layer="94" ratio="15">G</text>
<text x="-20.812" y="-7.668" size="1.016" layer="94" ratio="15">DP</text>
<pin name="A" x="-17.78" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="B" x="-15.24" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="C" x="-12.7" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="D" x="-10.16" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="E" x="-7.62" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="F" x="-5.08" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="G" x="-2.54" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="DP" x="0" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="D1" x="10.16" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="D2" x="12.7" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="D3" x="15.24" y="-17.78" visible="pin" length="short" rot="R90"/>
<pin name="D4" x="17.78" y="-17.78" visible="pin" length="short" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="7-SEGMENT-4DIGIT-20MM" prefix="DISP">
<description>&lt;h3&gt;4 Digit 7 Segment Display&lt;/h3&gt;
&lt;p&gt;This is a part for the 4-Digit 7-segment LEDs from Young Sun.  Different from the Digikey LEDs, all 16 pins are used.
&lt;br&gt;
The LONGPADS Variant is used for a test jig, but you may like it because there is more pad to solder to. If you are concerned about routing space, the standard works just fine though.&lt;/p&gt;
&lt;h4&gt;Datasheets:&lt;/h4&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/LED/1LEDREDCC.pdf"&gt;Red&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/LED/1LEDYELCC.pdf"&gt;Yellow&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/LED/1LEDGRNCC.pdf"&gt;Green&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/LED/1LEDBLUCC.pdf"&gt;Blue&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;&lt;a href="http://cdn.sparkfun.com/datasheets/Components/LED/1LEDWHTCC.pdf"&gt;White&lt;/a&gt;&lt;/li&gt;
&lt;/ul&gt;
&lt;h4&gt;SparkFun Products&lt;/h4&gt;
&lt;ul&gt;
&lt;h5&gt;Raw Components&lt;/h5&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11405"&gt;7-Segment Display - 20mm (Red)&lt;/a&gt; (COM-11405)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11406"&gt;7-Segment Display - 20mm (Yellow)&lt;/a&gt; (COM-11406)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11407"&gt;7-Segment Display - 20mm (Green)&lt;/a&gt; (COM-11407)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11408"&gt;7-Segment Display - 20mm (Blue)&lt;/a&gt; (COM-11408)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11409"&gt;7-Segment Display - 20mm (White)&lt;/a&gt; (COM-11409)&lt;/li&gt;
&lt;h5&gt;Serial Controlled&lt;/h5&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11644"&gt;SparkFun OpenSegment Serial Display - 20mm (Red)&lt;/a&gt; (COM-11644)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11645"&gt;SparkFun OpenSegment Serial Display - 20mm (Yellow)&lt;/a&gt; (COM-11645)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11646"&gt;SparkFun OpenSegment Serial Display - 20mm (Green)&lt;/a&gt; (COM-11646)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11647"&gt;SparkFun OpenSegment Serial Display - 20mm (Blue)&lt;/a&gt; (COM-11647)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/11648"&gt;SparkFun OpenSegment Serial Display - 20mm (White)&lt;/a&gt; (COM-11648)&lt;/li&gt;
&lt;h5&gt;Other&lt;/h5&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/13190"&gt;SparkFun OpenSegment Shield&lt;/a&gt; (DEV-13190)&lt;/li&gt;
&lt;/ul&gt;</description>
<gates>
<gate name="G$1" symbol="7-SEGMENT-4DIGIT-COUNTER" x="0" y="0"/>
</gates>
<devices>
<device name="-RED" package="7-SEGMENT-4DIGIT-20MM">
<connects>
<connect gate="G$1" pin="A" pad="11"/>
<connect gate="G$1" pin="B" pad="7"/>
<connect gate="G$1" pin="C" pad="4"/>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="D1" pad="12"/>
<connect gate="G$1" pin="D2" pad="9"/>
<connect gate="G$1" pin="D3" pad="8"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
<connect gate="G$1" pin="F" pad="10"/>
<connect gate="G$1" pin="G" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-11449" constant="no"/>
<attribute name="VALUE" value="ATA8041ARBJ " constant="no"/>
</technology>
</technologies>
</device>
<device name="-HP-BUBBLE" package="DIP12-HP-BUBBLE">
<connects>
<connect gate="G$1" pin="A" pad="12"/>
<connect gate="G$1" pin="B" pad="11"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="D" pad="8"/>
<connect gate="G$1" pin="D1" pad="1"/>
<connect gate="G$1" pin="D2" pad="10"/>
<connect gate="G$1" pin="D3" pad="4"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="5"/>
<connect gate="G$1" pin="E" pad="2"/>
<connect gate="G$1" pin="F" pad="9"/>
<connect gate="G$1" pin="G" pad="7"/>
</connects>
<technologies>
<technology name="">
<attribute name="STOREFRONT" value="COM-12710" constant="no"/>
</technology>
</technologies>
</device>
<device name="-YELLOW" package="7-SEGMENT-4DIGIT-20MM">
<connects>
<connect gate="G$1" pin="A" pad="11"/>
<connect gate="G$1" pin="B" pad="7"/>
<connect gate="G$1" pin="C" pad="4"/>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="D1" pad="12"/>
<connect gate="G$1" pin="D2" pad="9"/>
<connect gate="G$1" pin="D3" pad="8"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
<connect gate="G$1" pin="F" pad="10"/>
<connect gate="G$1" pin="G" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-11448" constant="no"/>
<attribute name="VALUE" value="ATA8041AY" constant="no"/>
</technology>
</technologies>
</device>
<device name="-GREEN" package="7-SEGMENT-4DIGIT-20MM">
<connects>
<connect gate="G$1" pin="A" pad="11"/>
<connect gate="G$1" pin="B" pad="7"/>
<connect gate="G$1" pin="C" pad="4"/>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="D1" pad="12"/>
<connect gate="G$1" pin="D2" pad="9"/>
<connect gate="G$1" pin="D3" pad="8"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
<connect gate="G$1" pin="F" pad="10"/>
<connect gate="G$1" pin="G" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-11447" constant="no"/>
<attribute name="VALUE" value="ATA8041AG" constant="no"/>
</technology>
</technologies>
</device>
<device name="-BLUE" package="7-SEGMENT-4DIGIT-20MM">
<connects>
<connect gate="G$1" pin="A" pad="11"/>
<connect gate="G$1" pin="B" pad="7"/>
<connect gate="G$1" pin="C" pad="4"/>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="D1" pad="12"/>
<connect gate="G$1" pin="D2" pad="9"/>
<connect gate="G$1" pin="D3" pad="8"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
<connect gate="G$1" pin="F" pad="10"/>
<connect gate="G$1" pin="G" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-11446" constant="no"/>
<attribute name="VALUE" value="ATA8041AB" constant="no"/>
</technology>
</technologies>
</device>
<device name="-WHITE" package="7-SEGMENT-4DIGIT-20MM">
<connects>
<connect gate="G$1" pin="A" pad="11"/>
<connect gate="G$1" pin="B" pad="7"/>
<connect gate="G$1" pin="C" pad="4"/>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="D1" pad="12"/>
<connect gate="G$1" pin="D2" pad="9"/>
<connect gate="G$1" pin="D3" pad="8"/>
<connect gate="G$1" pin="D4" pad="6"/>
<connect gate="G$1" pin="DP" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
<connect gate="G$1" pin="F" pad="10"/>
<connect gate="G$1" pin="G" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-11445" constant="no"/>
<attribute name="VALUE" value="ATA8041AW" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-PowerSymbols">
<description>&lt;h3&gt;SparkFun Power Symbols&lt;/h3&gt;
This library contains power, ground, and voltage-supply symbols.
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
</packages>
<symbols>
<symbol name="DGND">
<description>&lt;h3&gt;Digital Ground Supply&lt;/h3&gt;</description>
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
<text x="0" y="-0.254" size="1.778" layer="96" align="top-center">&gt;VALUE</text>
</symbol>
<symbol name="VCC">
<description>&lt;h3&gt;VCC Voltage Supply&lt;/h3&gt;</description>
<wire x1="0.762" y1="1.27" x2="0" y2="2.54" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="-0.762" y2="1.27" width="0.254" layer="94"/>
<pin name="VCC" x="0" y="0" visible="off" length="short" direction="sup" rot="R90"/>
<text x="0" y="2.794" size="1.778" layer="96" align="bottom-center">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="GND">
<description>&lt;h3&gt;Ground Supply Symbol&lt;/h3&gt;
&lt;p&gt;Generic signal ground supply symbol.&lt;/p&gt;</description>
<gates>
<gate name="1" symbol="DGND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" prefix="SUPPLY">
<description>&lt;h3&gt;VCC Voltage Supply&lt;/h3&gt;
&lt;p&gt;Positive voltage supply (traditionally for a BJT device, C=collector).&lt;/p&gt;</description>
<gates>
<gate name="G$1" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-Boards">
<description>&lt;h3&gt;SparkFun Electronics' preferred foot prints&lt;/h3&gt;
This library contains footprints for SparkFun breakout boards, microcontrollers (Arduino, Particle, Teensy, etc.),  breadboards, non-RF modules, etc.
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="TEENSY-3.1">
<description>&lt;h3&gt;Teensy 3 Footprint&lt;/h3&gt;
&lt;p&gt;Specifications:
&lt;ul&gt;&lt;li&gt;Pin count:37&lt;/li&gt;
&lt;li&gt;Pin pitch: 0.1"&lt;/li&gt;
&lt;li&gt;Area:0.7x1.4"&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;
&lt;p&gt;Example device(s):
&lt;ul&gt;&lt;li&gt;Teensy 3.1&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;</description>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="-17.78" x2="3.81" y2="-17.78" width="0.127" layer="51"/>
<pad name="13" x="-7.62" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="14" x="-7.62" y="13.97" drill="1.016" diameter="1.8796"/>
<pad name="15" x="-7.62" y="11.43" drill="1.016" diameter="1.8796"/>
<pad name="16" x="-7.62" y="8.89" drill="1.016" diameter="1.8796"/>
<pad name="17" x="-7.62" y="6.35" drill="1.016" diameter="1.8796"/>
<pad name="18" x="-7.62" y="3.81" drill="1.016" diameter="1.8796"/>
<pad name="19" x="-7.62" y="1.27" drill="1.016" diameter="1.8796"/>
<pad name="20" x="-7.62" y="-1.27" drill="1.016" diameter="1.8796"/>
<pad name="21" x="-7.62" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="22" x="-7.62" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="23" x="-7.62" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="3V" x="-7.62" y="-11.43" drill="1.016" diameter="1.8796"/>
<pad name="AGND" x="-7.62" y="-13.97" drill="1.016" diameter="1.8796"/>
<pad name="VIN" x="-7.62" y="-16.51" drill="1.016" diameter="1.8796"/>
<pad name="12" x="7.62" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="11" x="7.62" y="13.97" drill="1.016" diameter="1.8796"/>
<pad name="10" x="7.62" y="11.43" drill="1.016" diameter="1.8796"/>
<pad name="9" x="7.62" y="8.89" drill="1.016" diameter="1.8796"/>
<pad name="8" x="7.62" y="6.35" drill="1.016" diameter="1.8796"/>
<pad name="7" x="7.62" y="3.81" drill="1.016" diameter="1.8796"/>
<pad name="6" x="7.62" y="1.27" drill="1.016" diameter="1.8796"/>
<pad name="5" x="7.62" y="-1.27" drill="1.016" diameter="1.8796"/>
<pad name="4" x="7.62" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="3" x="7.62" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="2" x="7.62" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="1" x="7.62" y="-11.43" drill="1.016" diameter="1.8796"/>
<pad name="0" x="7.62" y="-13.97" drill="1.016" diameter="1.8796"/>
<pad name="GND1" x="7.62" y="-16.51" drill="1.016" diameter="1.8796"/>
<pad name="A14/DAC" x="-5.08" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="PROG" x="-2.54" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="GND" x="0" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="3.3V" x="2.54" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="VBAT" x="5.08" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="A11" x="-5.08" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="A10" x="-5.08" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="AREF" x="-5.08" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="VUSB" x="-5.08" y="-13.97" drill="1.016" diameter="1.8796"/>
<text x="0" y="18.034" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-19.304" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<wire x1="3.81" y1="-17.78" x2="-3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-19.05" x2="3.81" y2="-19.05" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-19.05" x2="-3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="3.81" y1="-19.05" x2="3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.2032" layer="21"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="22"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="22"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.2032" layer="21"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="22"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.2032" layer="21"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.2032" layer="21"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="22"/>
<text x="-5.3975" y="-17.4625" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">VIN</text>
<text x="-5.3975" y="-12.3825" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">VIN</text>
<text x="-2.8575" y="-15.24" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">VUSB</text>
<text x="-2.8575" y="-10.795" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">AREF</text>
<text x="-2.8575" y="-7.62" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A10</text>
<text x="-2.8575" y="-4.7625" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A11</text>
<text x="-5.3975" y="-1.905" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A6</text>
<text x="-5.3975" y="0.635" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A5</text>
<text x="-5.3975" y="3.175" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A4</text>
<text x="-5.3975" y="5.715" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A3</text>
<text x="-5.3975" y="8.255" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A2</text>
<text x="-5.3975" y="10.795" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A1</text>
<text x="-5.3975" y="13.335" size="0.8128" layer="21" font="vector" ratio="15" rot="R90">A0</text>
<text x="6.35" y="14.605" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">11</text>
<text x="6.35" y="12.065" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">10</text>
<text x="6.35" y="9.2075" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">9</text>
<text x="6.35" y="6.6675" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">8</text>
<text x="6.35" y="4.445" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">7</text>
<text x="6.35" y="1.5875" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">6</text>
<text x="6.35" y="-0.9525" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">5</text>
<text x="6.35" y="-3.4925" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">4</text>
<text x="6.35" y="-6.0325" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">3</text>
<text x="6.35" y="-8.5725" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">2</text>
<text x="6.35" y="-11.1125" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">1</text>
<text x="6.35" y="-13.6525" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">0</text>
<text x="6.35" y="-15.24" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">GND</text>
<text x="-9.2075" y="-5.3975" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">A8</text>
<text x="-9.2075" y="-7.9375" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">A9</text>
<text x="-9.2075" y="-12.7" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">AGND</text>
<text x="-1.905" y="14.9225" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">PROG</text>
<text x="0.3175" y="14.9225" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">GND</text>
<text x="2.8575" y="14.9225" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">3.3V</text>
<text x="5.3975" y="14.9225" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">VBAT</text>
<text x="-4.445" y="14.9225" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">A14</text>
<text x="-9.2075" y="-2.8575" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">A7</text>
<text x="-9.2075" y="17.145" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">13</text>
<text x="9.2075" y="15.875" size="0.8128" layer="21" font="vector" ratio="15" rot="R270" align="bottom-right">12</text>
<text x="-9.2075" y="-10.4775" size="0.8128" layer="21" font="vector" ratio="15" rot="R90" align="bottom-right">3V</text>
<text x="-9.2075" y="-5.3975" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">A8</text>
<text x="-9.2075" y="-7.9375" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">A9</text>
<text x="-9.2075" y="-12.7" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">AGND</text>
<text x="-9.2075" y="-2.8575" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">A7</text>
<text x="-9.2075" y="17.145" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">13</text>
<text x="-9.2075" y="-10.4775" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270">3V</text>
<text x="-5.3975" y="-12.3825" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">VIN</text>
<text x="-2.8575" y="-15.24" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">VUSB</text>
<text x="-5.3975" y="-17.4625" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">VIN</text>
<text x="-2.8575" y="-10.795" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">AREF</text>
<text x="-2.8575" y="-7.62" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A10</text>
<text x="-2.8575" y="-4.7625" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A11</text>
<text x="-5.3975" y="-1.905" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A6</text>
<text x="-5.3975" y="0.635" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A5</text>
<text x="-5.3975" y="3.175" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A4</text>
<text x="-5.3975" y="5.715" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A3</text>
<text x="-5.3975" y="8.255" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A2</text>
<text x="-5.3975" y="10.795" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A1</text>
<text x="-4.445" y="14.9225" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">A14</text>
<text x="-1.905" y="14.9225" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">PROG</text>
<text x="0.3175" y="14.9225" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">GND</text>
<text x="2.8575" y="14.9225" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">3.3V</text>
<text x="5.3975" y="14.9225" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">VBAT</text>
<text x="9.2075" y="15.875" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90">12</text>
<text x="6.35" y="-15.24" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">GND</text>
<text x="6.35" y="-13.6525" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">0</text>
<text x="6.35" y="-11.1125" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">1</text>
<text x="6.35" y="-8.5725" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">2</text>
<text x="6.35" y="-6.0325" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">3</text>
<text x="6.35" y="-3.4925" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">4</text>
<text x="6.35" y="-0.9525" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">5</text>
<text x="6.35" y="1.5875" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">6</text>
<text x="6.35" y="4.445" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">7</text>
<text x="6.35" y="6.6675" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">8</text>
<text x="6.35" y="9.2075" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">9</text>
<text x="6.35" y="12.065" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">10</text>
<text x="6.35" y="14.605" size="0.8128" layer="22" font="vector" ratio="15" rot="MR90" align="top-right">11</text>
<text x="-5.3975" y="13.335" size="0.8128" layer="22" font="vector" ratio="15" rot="MR270" align="bottom-right">A0</text>
</package>
<package name="TEENSY-3.1_NOSILK">
<description>&lt;h3&gt;Teensy 3 Footprint- No Silk Label on Pins&lt;/h3&gt;
&lt;p&gt;Specifications:
&lt;ul&gt;&lt;li&gt;Pin count:37&lt;/li&gt;
&lt;li&gt;Pin pitch: 0.1"&lt;/li&gt;
&lt;li&gt;Area:0.7x1.4"&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;
&lt;p&gt;Example device(s):
&lt;ul&gt;&lt;li&gt;Teensy 3.1&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;</description>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="-17.78" x2="3.81" y2="-17.78" width="0.127" layer="51"/>
<pad name="13" x="-7.62" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="14" x="-7.62" y="13.97" drill="1.016" diameter="1.8796"/>
<pad name="15" x="-7.62" y="11.43" drill="1.016" diameter="1.8796"/>
<pad name="16" x="-7.62" y="8.89" drill="1.016" diameter="1.8796"/>
<pad name="17" x="-7.62" y="6.35" drill="1.016" diameter="1.8796"/>
<pad name="18" x="-7.62" y="3.81" drill="1.016" diameter="1.8796"/>
<pad name="19" x="-7.62" y="1.27" drill="1.016" diameter="1.8796"/>
<pad name="20" x="-7.62" y="-1.27" drill="1.016" diameter="1.8796"/>
<pad name="21" x="-7.62" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="22" x="-7.62" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="23" x="-7.62" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="3V" x="-7.62" y="-11.43" drill="1.016" diameter="1.8796"/>
<pad name="AGND" x="-7.62" y="-13.97" drill="1.016" diameter="1.8796"/>
<pad name="VIN" x="-7.62" y="-16.51" drill="1.016" diameter="1.8796"/>
<pad name="12" x="7.62" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="11" x="7.62" y="13.97" drill="1.016" diameter="1.8796"/>
<pad name="10" x="7.62" y="11.43" drill="1.016" diameter="1.8796"/>
<pad name="9" x="7.62" y="8.89" drill="1.016" diameter="1.8796"/>
<pad name="8" x="7.62" y="6.35" drill="1.016" diameter="1.8796"/>
<pad name="7" x="7.62" y="3.81" drill="1.016" diameter="1.8796"/>
<pad name="6" x="7.62" y="1.27" drill="1.016" diameter="1.8796"/>
<pad name="5" x="7.62" y="-1.27" drill="1.016" diameter="1.8796"/>
<pad name="4" x="7.62" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="3" x="7.62" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="2" x="7.62" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="1" x="7.62" y="-11.43" drill="1.016" diameter="1.8796"/>
<pad name="0" x="7.62" y="-13.97" drill="1.016" diameter="1.8796"/>
<pad name="GND1" x="7.62" y="-16.51" drill="1.016" diameter="1.8796"/>
<pad name="A14/DAC" x="-5.08" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="PROG" x="-2.54" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="GND" x="0" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="3.3V" x="2.54" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="VBAT" x="5.08" y="16.51" drill="1.016" diameter="1.8796"/>
<pad name="A11" x="-5.08" y="-3.81" drill="1.016" diameter="1.8796"/>
<pad name="A10" x="-5.08" y="-6.35" drill="1.016" diameter="1.8796"/>
<pad name="AREF" x="-5.08" y="-8.89" drill="1.016" diameter="1.8796"/>
<pad name="VUSB" x="-5.08" y="-13.97" drill="1.016" diameter="1.8796"/>
<text x="0" y="18.034" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-19.304" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<wire x1="3.81" y1="-17.78" x2="-3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-19.05" x2="3.81" y2="-19.05" width="0.127" layer="51"/>
<wire x1="-3.81" y1="-19.05" x2="-3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="3.81" y1="-19.05" x2="3.81" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="22"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="22"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="21"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="22"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="21"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="21"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="22"/>
</package>
</packages>
<symbols>
<symbol name="TEENSY-31">
<description>&lt;h3&gt;Teensy 3.1&lt;/h3&gt;
&lt;p&gt;Breaks out all the PTH pins on the Teensy 3.1. Does not break out all of the additional SMD pins on the underside of the Teensy, or the external crystal pins. &lt;/p&gt;</description>
<wire x1="-15.24" y1="-33.02" x2="-15.24" y2="30.48" width="0.254" layer="94"/>
<wire x1="-15.24" y1="30.48" x2="12.7" y2="30.48" width="0.254" layer="94"/>
<wire x1="12.7" y1="30.48" x2="12.7" y2="-33.02" width="0.254" layer="94"/>
<wire x1="12.7" y1="-33.02" x2="-15.24" y2="-33.02" width="0.254" layer="94"/>
<text x="-5.08" y="30.734" size="1.778" layer="95" font="vector">&gt;NAME</text>
<text x="-5.08" y="-33.274" size="1.778" layer="96" font="vector" align="top-left">&gt;VALUE</text>
<pin name="GND1" x="-17.78" y="-30.48" length="short" direction="pwr"/>
<pin name="GND" x="-17.78" y="-27.94" length="short" direction="pwr"/>
<pin name="VBAT" x="-17.78" y="-15.24" length="short" direction="pwr"/>
<pin name="3.3V" x="-17.78" y="-12.7" length="short" direction="pwr"/>
<pin name="3V" x="-17.78" y="-10.16" length="short" direction="pwr"/>
<pin name="VIN" x="-17.78" y="-7.62" length="short" direction="pwr"/>
<pin name="AGND" x="-17.78" y="-25.4" length="short" direction="pwr"/>
<pin name="A0/14" x="-17.78" y="27.94" length="short"/>
<pin name="A1/15" x="-17.78" y="25.4" length="short"/>
<pin name="A2/16" x="-17.78" y="22.86" length="short"/>
<pin name="A3/17" x="-17.78" y="20.32" length="short"/>
<pin name="A4/18/SDA0" x="15.24" y="22.86" length="short" rot="R180"/>
<pin name="A5/19/SCL0" x="15.24" y="20.32" length="short" rot="R180"/>
<pin name="A6/20*" x="-17.78" y="12.7" length="short"/>
<pin name="A7/21*" x="-17.78" y="10.16" length="short"/>
<pin name="A8/22*" x="-17.78" y="7.62" length="short"/>
<pin name="A9/23*" x="-17.78" y="5.08" length="short"/>
<pin name="10*/TX2/CS" x="15.24" y="-30.48" length="short" rot="R180"/>
<pin name="11/DOUT" x="15.24" y="-27.94" length="short" rot="R180"/>
<pin name="12/DIN" x="15.24" y="-25.4" length="short" rot="R180"/>
<pin name="13/SCK" x="15.24" y="-22.86" length="short" rot="R180"/>
<pin name="A10" x="-17.78" y="2.54" length="short"/>
<pin name="A11" x="-17.78" y="0" length="short"/>
<pin name="A14/DAC" x="-17.78" y="-2.54" length="short"/>
<pin name="AREF" x="-17.78" y="-20.32" length="short"/>
<pin name="VUSB" x="-17.78" y="-17.78" length="short" direction="pwr"/>
<pin name="8/TX3" x="15.24" y="-7.62" length="short" rot="R180"/>
<pin name="7/RX3" x="15.24" y="-5.08" length="short" rot="R180"/>
<pin name="6*" x="15.24" y="-2.54" length="short" rot="R180"/>
<pin name="5*" x="15.24" y="0" length="short" rot="R180"/>
<pin name="4*" x="15.24" y="2.54" length="short" rot="R180"/>
<pin name="3*" x="15.24" y="5.08" length="short" rot="R180"/>
<pin name="2" x="15.24" y="7.62" length="short" rot="R180"/>
<pin name="1/TX1" x="15.24" y="10.16" length="short" rot="R180"/>
<pin name="0/RX1" x="15.24" y="12.7" length="short" rot="R180"/>
<pin name="9*/RX2" x="15.24" y="-10.16" length="short" rot="R180"/>
<pin name="PROG" x="15.24" y="27.94" length="short" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TEENSY-3.1" prefix="B">
<description>&lt;h3&gt;Teensy 3+ Footprint&lt;/h3&gt;
&lt;p&gt;Mechanical footprint with connectors for Teensy Development Board with ARM Cortex.&lt;/p&gt;

&lt;b&gt;&lt;p&gt;SparkFun Products:&lt;/b&gt;
&lt;ul&gt;&lt;li&gt;&lt;a href=”https://www.sparkfun.com/products/13736"&gt;Teensy 3.2&lt;/a&gt;&lt;/li&gt;
&lt;li&gt;&lt;a href=”https://www.sparkfun.com/products/13305"&gt;Teensy LC&lt;/a&gt;&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;</description>
<gates>
<gate name="G$1" symbol="TEENSY-31" x="0" y="0"/>
</gates>
<devices>
<device name="SILK" package="TEENSY-3.1">
<connects>
<connect gate="G$1" pin="0/RX1" pad="0"/>
<connect gate="G$1" pin="1/TX1" pad="1"/>
<connect gate="G$1" pin="10*/TX2/CS" pad="10"/>
<connect gate="G$1" pin="11/DOUT" pad="11"/>
<connect gate="G$1" pin="12/DIN" pad="12"/>
<connect gate="G$1" pin="13/SCK" pad="13"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3*" pad="3"/>
<connect gate="G$1" pin="3.3V" pad="3.3V"/>
<connect gate="G$1" pin="3V" pad="3V"/>
<connect gate="G$1" pin="4*" pad="4"/>
<connect gate="G$1" pin="5*" pad="5"/>
<connect gate="G$1" pin="6*" pad="6"/>
<connect gate="G$1" pin="7/RX3" pad="7"/>
<connect gate="G$1" pin="8/TX3" pad="8"/>
<connect gate="G$1" pin="9*/RX2" pad="9"/>
<connect gate="G$1" pin="A0/14" pad="14"/>
<connect gate="G$1" pin="A1/15" pad="15"/>
<connect gate="G$1" pin="A10" pad="A10"/>
<connect gate="G$1" pin="A11" pad="A11"/>
<connect gate="G$1" pin="A14/DAC" pad="A14/DAC"/>
<connect gate="G$1" pin="A2/16" pad="16"/>
<connect gate="G$1" pin="A3/17" pad="17"/>
<connect gate="G$1" pin="A4/18/SDA0" pad="18"/>
<connect gate="G$1" pin="A5/19/SCL0" pad="19"/>
<connect gate="G$1" pin="A6/20*" pad="20"/>
<connect gate="G$1" pin="A7/21*" pad="21"/>
<connect gate="G$1" pin="A8/22*" pad="22"/>
<connect gate="G$1" pin="A9/23*" pad="23"/>
<connect gate="G$1" pin="AGND" pad="AGND"/>
<connect gate="G$1" pin="AREF" pad="AREF"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="GND1" pad="GND1"/>
<connect gate="G$1" pin="PROG" pad="PROG"/>
<connect gate="G$1" pin="VBAT" pad="VBAT"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
<connect gate="G$1" pin="VUSB" pad="VUSB"/>
</connects>
<technologies>
<technology name="">
<attribute name="SF_ID" value="DEV-12646" constant="no"/>
</technology>
</technologies>
</device>
<device name="NOSILK" package="TEENSY-3.1_NOSILK">
<connects>
<connect gate="G$1" pin="0/RX1" pad="0"/>
<connect gate="G$1" pin="1/TX1" pad="1"/>
<connect gate="G$1" pin="10*/TX2/CS" pad="10"/>
<connect gate="G$1" pin="11/DOUT" pad="11"/>
<connect gate="G$1" pin="12/DIN" pad="12"/>
<connect gate="G$1" pin="13/SCK" pad="13"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3*" pad="3"/>
<connect gate="G$1" pin="3.3V" pad="3.3V"/>
<connect gate="G$1" pin="3V" pad="3V"/>
<connect gate="G$1" pin="4*" pad="4"/>
<connect gate="G$1" pin="5*" pad="5"/>
<connect gate="G$1" pin="6*" pad="6"/>
<connect gate="G$1" pin="7/RX3" pad="7"/>
<connect gate="G$1" pin="8/TX3" pad="8"/>
<connect gate="G$1" pin="9*/RX2" pad="9"/>
<connect gate="G$1" pin="A0/14" pad="14"/>
<connect gate="G$1" pin="A1/15" pad="15"/>
<connect gate="G$1" pin="A10" pad="A10"/>
<connect gate="G$1" pin="A11" pad="A11"/>
<connect gate="G$1" pin="A14/DAC" pad="A14/DAC"/>
<connect gate="G$1" pin="A2/16" pad="16"/>
<connect gate="G$1" pin="A3/17" pad="17"/>
<connect gate="G$1" pin="A4/18/SDA0" pad="18"/>
<connect gate="G$1" pin="A5/19/SCL0" pad="19"/>
<connect gate="G$1" pin="A6/20*" pad="20"/>
<connect gate="G$1" pin="A7/21*" pad="21"/>
<connect gate="G$1" pin="A8/22*" pad="22"/>
<connect gate="G$1" pin="A9/23*" pad="23"/>
<connect gate="G$1" pin="AGND" pad="AGND"/>
<connect gate="G$1" pin="AREF" pad="AREF"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="GND1" pad="GND1"/>
<connect gate="G$1" pin="PROG" pad="PROG"/>
<connect gate="G$1" pin="VBAT" pad="VBAT"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
<connect gate="G$1" pin="VUSB" pad="VUSB"/>
</connects>
<technologies>
<technology name="">
<attribute name="SF_ID" value="DEV-12646" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="C1" library="SparkFun-Capacitors" deviceset="1.0UF" device="-0805-25V-10%" value="1.0uF"/>
<part name="S1" library="SparkFun-Switches" deviceset="REED_SWITCH" device="-PTH-GLASS"/>
<part name="S2" library="SparkFun-Switches" deviceset="MOMENTARY-SWITCH-SPST" device="-PTH-6.0MM"/>
<part name="S3" library="SparkFun-Switches" deviceset="ENCODER-SWITCH" device=""/>
<part name="DISP1" library="SparkFun-LED" deviceset="7-SEGMENT-4DIGIT-20MM" device="-RED" value="ATA8041ARBJ "/>
<part name="GND1" library="SparkFun-PowerSymbols" deviceset="GND" device=""/>
<part name="SUPPLY1" library="SparkFun-PowerSymbols" deviceset="VCC" device=""/>
<part name="B1" library="SparkFun-Boards" deviceset="TEENSY-3.1" device="SILK"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="C1" gate="G$1" x="35.56" y="48.26"/>
<instance part="S1" gate="G$1" x="-2.54" y="114.3"/>
<instance part="S2" gate="G$1" x="-45.72" y="27.94"/>
<instance part="S3" gate="G$1" x="-5.08" y="27.94"/>
<instance part="DISP1" gate="G$1" x="-35.56" y="60.96"/>
<instance part="GND1" gate="1" x="48.26" y="10.16"/>
<instance part="SUPPLY1" gate="G$1" x="38.1" y="7.62"/>
<instance part="B1" gate="G$1" x="78.74" y="40.64"/>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
