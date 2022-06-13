set OSPL_HOME=C:/developpement/C++/libs/opensplice/HDE/x86_64.win64

%OSPL_HOME%\bin\idlpp -l cs -d ../../Scripts/DDS/Topics ./common.idl
%OSPL_HOME%\bin\idlpp -l cs -d ../../Scripts/DDS/Topics ./entity.idl

pause