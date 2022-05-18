# diffbot


+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
**********  XRCE DDS AGENT INSTALL  *********

https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#:~:text=DDS%20Agent%20page.-,Installation%20from%20Snap%20package,the%20master%20branch%20on%20GitHub.

>> Clone the project from GitHub:

$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
$ cd Micro-XRCE-DDS-Agent
$ mkdir build && cd build


>> On Linux, inside of the build folder, execute the following commands:

$ cmake ..
$ make
$ sudo make install


->> running cmd
cd /usr/local/bin && MicroXRCEAgent serial --dev /dev/ttyS0


+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
