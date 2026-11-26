; Inno Setup Script for pyCub Studio (Per-User Windows x64 Installer — No Admin Required)
#ifndef MyAppVersion
  #define MyAppVersion "1.4.0"
#endif

[Setup]
AppId={{8E9A74B1-4F2C-4D3B-9E18-7C65B92A14F0}
AppName=pyCub Studio
AppVersion={#MyAppVersion}
AppPublisher=CTU in Prague - Robotics
DefaultDirName={autopf}\pyCub Studio
DefaultGroupName=pyCub Studio
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=..\out
OutputBaseFilename=pyCub-Setup-{#MyAppVersion}-Windows-x64
SetupIconFile=..\..\studio\assets\icons\pycub.ico
Compression=lzma2/fast
SolidCompression=yes
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "czech"; MessagesFile: "compiler:Languages\Czech.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "..\work\dist\pyCub\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\pyCub Studio"; Filename: "{app}\pyCub.exe"
Name: "{autodesktop}\pyCub Studio"; Filename: "{app}\pyCub.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\pyCub.exe"; Description: "{cm:LaunchProgram,pyCub Studio}"; Flags: nowait postinstall skipifsilent
