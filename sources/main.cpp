#include <pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>
#include "pubSysCls.h"
#include <iostream>

namespace py = pybind11;


#define ACC_LIM_RPM_PER_SEC	3000
#define VEL_LIM_RPM			200
#define TIME_TILL_TIMEOUT	10000

class ClearpathTeknicDriver {

private:
    sFnd::SysManager* sysManager;
    sFnd::IPort* myPort;
    sFnd::INode* theNode;
    size_t portCount;

public:

    ClearpathTeknicDriver()
    {
        sysManager = NULL;
        myPort = NULL;
        theNode = NULL;
    }

    ~ClearpathTeknicDriver() {
        delete sysManager, myPort, theNode;
    }

    py::tuple connect()
    {
        portCount = 0;
        std::vector<std::string> comHubPorts;
        std::vector<int> nodesCount;
        std::vector<std::string> nodesNames;
        sysManager = sFnd::SysManager::Instance();
        try
        {
            sFnd::SysManager::FindComHubPorts(comHubPorts);
            printf("MotionController: Found %i SC Hubs\n", int(comHubPorts.size()));
            for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) 
            {
                sysManager->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
                                                // with COM portnum (as seen in device manager)
            }
            if (comHubPorts.size() < 1)
            {
                std::cout << "Unable to locate SC hub port\n" << std::endl;
                return py::make_tuple(false, comHubPorts.size(), nodesCount, nodesNames, "no ports found"); //This terminates the main program
            }

            //Open all the ports
            sysManager->PortsOpen(portCount);
            
            for (size_t i = 0; i < portCount; i++) {
                myPort = &sysManager->Ports(i);
                printf("MotionController: Port[%d]: state=%d, nodes=%d\n",
                    myPort->NetNumber(), myPort->OpenState(), myPort->NodeCount());
            
                //myPort = &sysManager->Ports(0);
                //printf("MotionController: Port[%d]: state=%d, nodes=%d\n",
                //    myPort->NetNumber(), myPort->OpenState(), myPort->NodeCount());

                //Once the code gets past this point, it can be assumed that the Port has been opened without issue
                //Now we can get a reference to our port object which we will use to access the node objects
                nodesCount.push_back(myPort->NodeCount());
                for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
                    theNode = &myPort->Nodes(iNode);
                    nodesNames.push_back(theNode->Info.UserID.Value());
                    theNode->EnableReq(false);				//Ensure Node is disabled before loading config file
                    sysManager->Delay(200);
                    //The following statements will attempt to enable the node.  First,
                    // any shutdowns or NodeStops are cleared, finally the node is enabled
                    theNode->Status.AlertsClear();					//Clear Alerts on node
                    theNode->Motion.NodeStopClear();	//Clear Nodestops on Node
                    theNode->EnableReq(true);					//Enable node
                    //At this point the node is enabled
                    printf("MotionController: Node \t%zi enabled\n", iNode);
                    double timeout = sysManager->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
                                                                                //This will loop checking on the Real time values of the node's Ready status
                    while (!theNode->Motion.IsReady()) {
                        if (sysManager->TimeStampMsec() > timeout) {
                            printf("MotionController: Error: Timed out waiting for Node %d to enable\n", 0);
                            return py::make_tuple(false, comHubPorts.size(), nodesCount, nodesNames, "timeout");
                        }
                    }
                }
            }
            printf("MotionController: Connected");
            return py::make_tuple(true, comHubPorts.size(), nodesCount, nodesNames, "good job!");
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to disable Nodes n\n");
            //This statement will print the address of the error, the error code (defined by the mnErr class), 
            //as well as the corresponding error message.
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return py::make_tuple(false, comHubPorts.size(), nodesCount, nodesNames, "exception");  //This terminates the main program
        }
    }

    int close()
    {
        printf("Disabling nodes, and closing port\n");
        //Disable Nodes
        for (size_t i = 0; i < portCount; i++) {
            myPort = &sysManager->Ports(i);
            for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
                // Create a shortcut reference for a node
                myPort->Nodes(iNode).EnableReq(false);
            }
        }
        // Close down the ports
        sysManager->PortsClose();
        return 1;
    }

	py::tuple home(int node=0, int port=0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            if (theNode->Motion.Homing.HomingValid())
            {
				theNode->Motion.PosnCommanded.Refresh();
				theNode->Motion.MoveWentDone();
                theNode->Motion.Homing.Initiate();
                printf("Node completed homing\n");
				return py::make_tuple(true, "success");
            }
            else {
                printf("MotionController: Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", 0);
                return py::make_tuple(false, "homing not valid");
            }
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to home\n");
            //This statement will print the address of the error, the error code (defined by the mnErr class), 
            //as well as the corresponding error message.
			return py::make_tuple(false, "exception");  //This terminates the main program
        }
    }



    py::tuple move(int cnts, float speed, bool is_relative, int node = 0, int port = 0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            theNode->Motion.PosnCommanded.Refresh();
            theNode->Motion.MoveWentDone();						//Clear the rising edge Move done register
            theNode->AccUnit(sFnd::INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
            theNode->VelUnit(sFnd::INode::RPM);						//Set the units for Velocity to RPM
            theNode->Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
            theNode->Motion.VelLimit = speed;				//Set Velocity Limit (RPM)

            size_t result = theNode->Motion.MovePosnStart(cnts, !is_relative);			//Execute  encoder count move

            double expected_time = theNode->Motion.MovePosnDurationMsec(cnts, !is_relative);

            return py::make_tuple(true, expected_time);
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to move\n");
            return py::make_tuple(false, -1);
        }
    }

    long long getPosition(int node = 0, int port = 0)
    {
        try
        {
            // Create a variable to return and refresh the position
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            long long scaledPosn;
            theNode->Motion.PosnCommanded.Refresh();

            scaledPosn = int64_t(theNode->Motion.PosnCommanded.Value());

            return scaledPosn;
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to get position\n");
            return 0;
        }
    }

    int stopNodeMotion(int node = 0, int port = 0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            theNode->Motion.NodeStop(STOP_TYPE_ABRUPT);
            return 1;
        }
        catch (...)
        {
            printf("Failed to stop motion\n");
            return 0;
        }        
    }

    int stopAllMotion()
    {
        try
        {
            for (size_t i = 0; i < portCount; i++) 
            {
                myPort = &sysManager->Ports(i);
                for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) 
                {
                    theNode = &myPort->Nodes(iNode);
                    theNode->Motion.NodeStop(STOP_TYPE_ABRUPT);
                }
            }
            return 1;
        }
        catch (...)
        {
            printf("Failed to stop motion\n");
            return 0;
        }
    }

    int enableNodeMotion(int node = 0, int port = 0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            theNode->Motion.NodeStopClear();
            return 1;
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to enable motion\n");
            return 0;
        }
    }

    int enableAllMotion()
    {
        try
        {
            for (size_t i = 0; i < portCount; i++) 
            {
                myPort = &sysManager->Ports(i);
                for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) 
                {
                    theNode = &myPort->Nodes(iNode);
                    theNode->Motion.NodeStopClear();
                }
            }
            return 1;
        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to enable motion\n");
            return 0;
        }
    }


    bool isMoving(int node = 0, int port = 0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            return !(theNode->Motion.MoveIsDone());

        }
        catch (sFnd::mnErr & theErr)
        {
            printf("Failed to get is moving\n");
            return false; 
        }
    }

    bool isHoming(int node = 0, int port = 0)
    {
        try
        {
            myPort = &sysManager->Ports(port);
            theNode = &myPort->Nodes(node);
            return theNode->Motion.Homing.IsHoming();

        }
        catch (sFnd::mnErr& theErr)
        {
            printf("Failed to get is moving\n");
            return false;
        }
    }
};


PYBIND11_MODULE(clearpathPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for Clearpath stepper motor Libraries
    )pbdoc";
    py::class_<ClearpathTeknicDriver> cteknic(m, "ClearpathTeknicDriver");
    cteknic.def(py::init<>());
    cteknic.def("connect", &ClearpathTeknicDriver::connect);
    cteknic.def("close", &ClearpathTeknicDriver::close);
    cteknic.def("home", &ClearpathTeknicDriver::home, py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("move", &ClearpathTeknicDriver::move, py::arg("cnts"), py::arg("speed"), py::arg("is_relative"), py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("getPosition", &ClearpathTeknicDriver::getPosition, py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("stopNodeMotion", &ClearpathTeknicDriver::stopNodeMotion, py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("stopAllMotion", &ClearpathTeknicDriver::stopAllMotion);
    cteknic.def("enableNodeMotion", &ClearpathTeknicDriver::enableNodeMotion, py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("enableAllMotion", &ClearpathTeknicDriver::enableAllMotion);
    cteknic.def("isMoving", &ClearpathTeknicDriver::isMoving, py::arg("node") = 0, py::arg("port") = 0);
    cteknic.def("isHoming", &ClearpathTeknicDriver::isHoming, py::arg("node") = 0, py::arg("port") = 0);

 
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



