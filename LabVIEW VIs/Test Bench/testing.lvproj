<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="19008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="testing.vi" Type="VI" URL="../../../src/x64/Debug/testing.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="user.lib" Type="Folder">
				<Item Name="Haptic Prog Interface.lvlib" Type="Library" URL="/&lt;userlib&gt;/Haptic Prog Interface/Haptic Prog Interface.lvlib"/>
			</Item>
			<Item Name="Haptic Programming Interface.dll" Type="Document" URL="../../../../../../../../../Program Files/National Instruments/LabVIEW 2019/src/x64/Debug/Haptic Programming Interface.dll"/>
			<Item Name="Haptic Programming Interface.dll" Type="Document" URL="../../../src/x64/Debug/Haptic Programming Interface.dll"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
