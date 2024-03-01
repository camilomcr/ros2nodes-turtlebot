#include "stubs.h"
#include <simPlusPlus/Lib.h>

#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

FuncTracer::FuncTracer(const std::string &f, int l)
    : f_(f),
      l_(l)
{
    sim::addLog(l_, f_ + " [enter]");
}

FuncTracer::~FuncTracer()
{
    sim::addLog(l_, f_ + " [leave]");
}

#ifndef NDEBUG

template<typename... Arguments>
void addStubsDebugLog(const std::string &fmt, Arguments&&... args)
{
    if(sim::isStackDebugEnabled())
    {
        auto msg = sim::util::sprintf(fmt, std::forward<Arguments>(args)...);
        sim::addLog(sim_verbosity_debug, "STUBS DEBUG: %s", msg);
    }
}

static void addStubsDebugStackDump(int stackHandle)
{
    if(sim::isStackDebugEnabled())
        sim::debugStack(stackHandle);
}

#else // RELEASE
#define addStubsDebugLog(...)
#define addStubsDebugStackDump(x)
#endif

#ifdef QT_COMPIL

Qt::HANDLE UI_THREAD = NULL;
Qt::HANDLE SIM_THREAD = NULL;

std::string threadNickname()
{
    Qt::HANDLE h = QThread::currentThreadId();
    if(h == UI_THREAD) return "UI";
    if(h == SIM_THREAD) return "SIM";
    std::stringstream ss;
    ss << h;
    return ss.str();
}

void uiThread()
{
    Qt::HANDLE h = QThread::currentThreadId();
    if(UI_THREAD != NULL && UI_THREAD != h)
        sim::addLog(sim_verbosity_warnings, "UI thread has already been set");
    UI_THREAD = h;
}

void simThread()
{
    Qt::HANDLE h = QThread::currentThreadId();
    if(SIM_THREAD != NULL && SIM_THREAD != h)
        sim::addLog(sim_verbosity_warnings, "SIM thread has already been set");
    SIM_THREAD = h;
}

#endif // QT_COMPIL

void readFromStack(int stack, bool *value, const ReadOptions &rdopt)
{
    bool v;
    if(sim::getStackBoolValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void readFromStack(int stack, int *value, const ReadOptions &rdopt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected int");
    }
}

void readFromStack(int stack, long *value, const ReadOptions &rdopt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected int");
    }
}

void readFromStack(int stack, float *value, const ReadOptions &rdopt)
{
    float v;
    if(sim::getStackFloatValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected float");
    }
}

void readFromStack(int stack, double *value, const ReadOptions &rdopt)
{
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void readFromStack(int stack, std::string *value, const ReadOptions &rdopt)
{
    std::string v;
    if(sim::getStackStringValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected string");
    }
}

template<typename T>
void readFromStack(int stack, boost::optional<T> *value, const ReadOptions &rdopt = {})
{
    if(sim::isStackValueNull(stack) == 1)
    {
        *value = boost::none;
        sim::popStackItem(stack, 1);
    }
    else
    {
        T v;
        readFromStack(stack, &v, rdopt); // will call sim::popStackItem() by itself
        *value = v;
    }
}

template<typename T>
void readFromStack(int stack, std::vector<T> *vec, const ReadOptions &rdopt = {})
{
    int sz = sim::getStackTableInfo(stack, 0);
    if(sz < 0)
        throw sim::exception("expected array (simGetStackTableInfo(stack, 0) returned %d)", sz);

    rdopt.validateTableSize(sz);

    int oldsz = sim::getStackSize(stack);
    sim::unfoldStackTable(stack);
    int sz1 = (sim::getStackSize(stack) - oldsz + 1) / 2;
    if(sz != sz1)
        throw std::runtime_error("simUnfoldStackTable unpacked more elements than simGetStackTableInfo reported");

    vec->resize(sz);

    for(int i = 0; i < sz; i++)
    {
        sim::moveStackItemToTop(stack, oldsz - 1);
        int j;
        readFromStack(stack, &j);
        sim::moveStackItemToTop(stack, oldsz - 1);
        if constexpr(std::is_same<T, bool>::value)
        {
            T v;
            readFromStack(stack, &v);
            (*vec)[i] = v;
        }
        else
        {
            readFromStack(stack, &vec->at(i));
        }
    }
}

template<typename T>
void readFromStack(int stack, std::vector<T> *vec, int (*f)(int, std::vector<T>*), const ReadOptions &rdopt = {})
{
    int sz = sim::getStackTableInfo(stack, 0);
    if(sz < 0)
        throw sim::exception("expected array (simGetStackTableInfo(stack, 0) returned %d)", sz);

    rdopt.validateTableSize(sz);

    int chk = sim::getStackTableInfo(stack, 2);
    if(chk != 1)
        throw sim::exception("table contains non-numbers (simGetStackTableInfo(stack, 2) returned %d)", chk);

    vec->resize(sz);

    int ret = f(stack, vec);
    if(ret != 1)
        throw sim::exception("readFunc error %d", ret);

    sim::popStackItem(stack, 1);
}

template<>
void readFromStack(int stack, std::vector<float> *vec, const ReadOptions &rdopt)
{
    readFromStack(stack, vec, sim::getStackFloatTable, rdopt);
}

template<>
void readFromStack(int stack, std::vector<double> *vec, const ReadOptions &rdopt)
{
    readFromStack(stack, vec, sim::getStackDoubleTable, rdopt);
}

template<>
void readFromStack(int stack, std::vector<int> *vec, const ReadOptions &rdopt)
{
    readFromStack(stack, vec, sim::getStackInt32Table, rdopt);
}

template<typename T>
void readFromStack(int stack, Grid<T> *grid, const ReadOptions &rdopt = {})
{
    try
    {
        int info = sim::getStackTableInfo(stack, 0);
        if(info != sim_stack_table_map && info != sim_stack_table_empty)
        {
            throw sim::exception("expected a map");
        }

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        std::set<std::string> requiredFields{"dims", "data"};

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            std::string key;
            readFromStack(stack, &key);

            sim::moveStackItemToTop(stack, oldsz - 1); // move value to top
            try
            {
                if(0) {}
                else if(key == "dims")
                {
                    readFromStack(stack, &grid->dims, ReadOptions().setBounds(0, 1, -1));
                }
                else if(key == "data")
                {
                    readFromStack(stack, &grid->data, ReadOptions());
                }
                else
                {
                    throw sim::exception("unexpected key");
                }
            }
            catch(std::exception &ex)
            {
                throw sim::exception("field '%s': %s", key, ex.what());
            }

            requiredFields.erase(key);
            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }

        for(const auto &field : requiredFields)
            throw sim::exception("missing required field '%s'", field);

        if(grid->dims.size() < 1)
            throw sim::exception("must have at least one dimension");

        size_t elemCount = 1;
        for(const int &i : grid->dims) elemCount *= i;
        if(grid->data.size() != elemCount)
            throw sim::exception("incorrect data length (expected %d elements)", elemCount);

        rdopt.validateSize(grid->dims);
    }
    catch(std::exception &ex)
    {
        throw sim::exception("readFromStack(Grid): %s", ex.what());
    }
}

void readFromStack(int stack, sim_ros2_time *value, const ReadOptions &rdopt)
{
    addStubsDebugLog("readFromStack(sim_ros2_time): begin reading...");
    addStubsDebugStackDump(stack);

    try
    {
        int info = sim::getStackTableInfo(stack, 0);
        if(info != sim_stack_table_map && info != sim_stack_table_empty)
        {
            throw sim::exception("expected a map");
        }

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        std::set<std::string> requiredFields{"sec", "nanosec"};

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            std::string key;
            readFromStack(stack, &key);

            sim::moveStackItemToTop(stack, oldsz - 1); // move value to top
            if(0) {}
            else if(key == "sec")
            {
                addStubsDebugLog("readFromStack(sim_ros2_time): reading field \"sec\" (int)...");
                try
                {
                    readFromStack(stack, &(value->sec));
                }
                catch(std::exception &ex)
                {
                    throw sim::exception("field 'sec': %s", ex.what());
                }
            }
            else if(key == "nanosec")
            {
                addStubsDebugLog("readFromStack(sim_ros2_time): reading field \"nanosec\" (int)...");
                try
                {
                    readFromStack(stack, &(value->nanosec));
                }
                catch(std::exception &ex)
                {
                    throw sim::exception("field 'nanosec': %s", ex.what());
                }
            }
            else
            {
                throw sim::exception("unexpected key: %s", key);
            }

            requiredFields.erase(key);
            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }

        for(const auto &field : requiredFields)
            throw sim::exception("missing required field '%s'", field);
    }
    catch(std::exception &ex)
    {
        throw sim::exception("readFromStack(sim_ros2_time): %s", ex.what());
    }

    addStubsDebugLog("readFromStack(sim_ros2_time): finished reading");
}

void writeToStack(const bool &value, int stack, const WriteOptions &wropt)
{
    sim::pushBoolOntoStack(stack, value);
}

void writeToStack(const int &value, int stack, const WriteOptions &wropt)
{
    sim::pushInt32OntoStack(stack, value);
}

void writeToStack(const long &value, int stack, const WriteOptions &wropt)
{
    if(value < std::numeric_limits<int>::max() || value > std::numeric_limits<int>::max())
        throw std::runtime_error("stack doesn't support (yet) int64 values");
    sim::pushInt32OntoStack(stack, static_cast<int>(value));
}

void writeToStack(const float &value, int stack, const WriteOptions &wropt)
{
    sim::pushFloatOntoStack(stack, value);
}

void writeToStack(const double &value, int stack, const WriteOptions &wropt)
{
    sim::pushDoubleOntoStack(stack, value);
}

void writeToStack(const std::string &value, int stack, const WriteOptions &wropt)
{
    sim::pushStringOntoStack(stack, value);
}

template<typename T>
void writeToStack(const boost::optional<T> &value, int stack, const WriteOptions &wropt = {})
{
    if(!value)
    {
        sim::pushNullOntoStack(stack);
        return;
    }

    writeToStack(*value, stack, wropt);
}

template<typename T>
void writeToStack(const std::vector<T> &vec, int stack, const WriteOptions &wropt = {})
{
    sim::pushTableOntoStack(stack);
    for(size_t i = 0; i < vec.size(); i++)
    {
        writeToStack(int(i + 1), stack);
        writeToStack(vec.at(i), stack);
        sim::insertDataIntoStackTable(stack);
    }
}

template<>
void writeToStack(const std::vector<float> &vec, int stack, const WriteOptions &wropt)
{
    sim::pushFloatTableOntoStack(stack, vec);
}

template<>
void writeToStack(const std::vector<double> &vec, int stack, const WriteOptions &wropt)
{
    sim::pushDoubleTableOntoStack(stack, vec);
}

template<>
void writeToStack(const std::vector<int> &vec, int stack, const WriteOptions &wropt)
{
    sim::pushInt32TableOntoStack(stack, vec);
}

template<typename T>
void writeToStack(const Grid<T> &grid, int stack, const WriteOptions &wropt = {})
{
    try
    {
        sim::pushTableOntoStack(stack);

        try
        {
            writeToStack(std::string{"dims"}, stack);
            writeToStack(grid.dims, stack);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("field 'dims': %s", ex.what());
        }
        try
        {
            writeToStack(std::string{"data"}, stack);
            writeToStack(grid.data, stack);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("field 'data': %s", ex.what());
        }
    }
    catch(std::exception &ex)
    {
        throw sim::exception("writeToStack(Grid): %s", ex.what());
    }
}

void writeToStack(const sim_ros2_time &value, int stack, const WriteOptions &wropt)
{
    addStubsDebugLog("writeToStack(sim_ros2_time): begin writing...");

    try
    {
        sim::pushTableOntoStack(stack);

        addStubsDebugLog("writeToStack(sim_ros2_time): writing field \"sec\" (int)...");
        try
        {
            writeToStack(std::string{"sec"}, stack);
            writeToStack(value.sec, stack);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("field 'sec': %s", ex.what());
        }
        addStubsDebugLog("writeToStack(sim_ros2_time): writing field \"nanosec\" (int)...");
        try
        {
            writeToStack(std::string{"nanosec"}, stack);
            writeToStack(value.nanosec, stack);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("field 'nanosec': %s", ex.what());
        }
    }
    catch(std::exception &ex)
    {
        throw sim::exception("writeToStack(sim_ros2_time): %s", ex.what());
    }

    addStubsDebugLog("writeToStack(sim_ros2_time): finished writing");
}

sim_ros2_time::sim_ros2_time()
{
}

void checkRuntimeVersion()
{
    int simVer = sim::programVersion();

    // version required by simStubsGen:
    int minVer = 4010000; // 4.1.0rev0
    if(simVer < minVer)
        throw sim::exception("requires at least %s (simStubsGen)", sim::versionString(minVer));

    // version required by plugin:
    if(simVer < SIM_REQUIRED_PROGRAM_VERSION_NB)
        throw sim::exception("requires at least %s", sim::versionString(SIM_REQUIRED_PROGRAM_VERSION_NB));

    // warn if the app older than the headers used to compile:
    if(simVer < SIM_PROGRAM_FULL_VERSION_NB)
        sim::addLog(sim_verbosity_warnings, "has been built for %s", sim::versionString(SIM_PROGRAM_FULL_VERSION_NB));
}

bool registerScriptStuff()
{
    try
    {
        checkRuntimeVersion();

        auto dbg = sim::getNamedBoolParam("simStubsGen.debug");
        if(dbg && *dbg)
            sim::enableStackDebug();

        try
        {
            sim::registerScriptVariable("simROS2", "require('simROS2-typecheck')", 0);
            sim::registerScriptVariable("_ROS2_latest_version", "math.max(_ROS2_latest_version or 0, 0)", 0);

            // register varables from enums:
            sim::registerScriptVariable("simROS2.clock_type", "{}", 0);
            sim::registerScriptVariable("simROS2.clock_type.ros", boost::lexical_cast<std::string>(sim_ros2_clock_ros), 0);
            sim::registerScriptVariable("simROS2.clock_type.system", boost::lexical_cast<std::string>(sim_ros2_clock_system), 0);
            sim::registerScriptVariable("simROS2.clock_type.steady", boost::lexical_cast<std::string>(sim_ros2_clock_steady), 0);
            sim::registerScriptVariable("simROS2.action_result_code", "{}", 0);
            sim::registerScriptVariable("simROS2.action_result_code.succeeded", boost::lexical_cast<std::string>(sim_ros2_action_result_code_succeeded), 0);
            sim::registerScriptVariable("simROS2.action_result_code.aborted", boost::lexical_cast<std::string>(sim_ros2_action_result_code_aborted), 0);
            sim::registerScriptVariable("simROS2.action_result_code.canceled", boost::lexical_cast<std::string>(sim_ros2_action_result_code_canceled), 0);
            sim::registerScriptVariable("simROS2.action_result_code.unknown", boost::lexical_cast<std::string>(sim_ros2_action_result_code_unknown), 0);
            sim::registerScriptVariable("simROS2.goal_response", "{}", 0);
            sim::registerScriptVariable("simROS2.goal_response.reject", boost::lexical_cast<std::string>(sim_ros2_goal_response_reject), 0);
            sim::registerScriptVariable("simROS2.goal_response.accept_and_execute", boost::lexical_cast<std::string>(sim_ros2_goal_response_accept_and_execute), 0);
            sim::registerScriptVariable("simROS2.goal_response.accept_and_defer", boost::lexical_cast<std::string>(sim_ros2_goal_response_accept_and_defer), 0);
            sim::registerScriptVariable("simROS2.cancel_response", "{}", 0);
            sim::registerScriptVariable("simROS2.cancel_response.reject", boost::lexical_cast<std::string>(sim_ros2_cancel_response_reject), 0);
            sim::registerScriptVariable("simROS2.cancel_response.accept", boost::lexical_cast<std::string>(sim_ros2_cancel_response_accept), 0);
            // register commands:
            sim::registerScriptCallbackFunction("simROS2.createSubscription@ROS2", "string subscriptionHandle=simROS2.createSubscription(string topicName,string topicType,string topicCallback,int queueSize=1)\n\nCreate a subscription to a topic.", createSubscription_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownSubscription@ROS2", "simROS2.shutdownSubscription(string subscriptionHandle)\n\nShutdown the subscription.", shutdownSubscription_callback);
            sim::registerScriptCallbackFunction("simROS2.subscriptionTreatUInt8ArrayAsString@ROS2", "simROS2.subscriptionTreatUInt8ArrayAsString(string subscriptionHandle)\n\nAfter calling this function, this subscription will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", subscriptionTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.createPublisher@ROS2", "string publisherHandle=simROS2.createPublisher(string topicName,string topicType,int queueSize=1,bool latch=false)\n\nCreate a topic publisher.", createPublisher_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownPublisher@ROS2", "simROS2.shutdownPublisher(string publisherHandle)\n\nShutdown the specified publisher.", shutdownPublisher_callback);
            sim::registerScriptCallbackFunction("simROS2.publisherTreatUInt8ArrayAsString@ROS2", "simROS2.publisherTreatUInt8ArrayAsString(string publisherHandle)\n\nAfter calling this function, this publisher will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", publisherTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.publish@ROS2", "simROS2.publish(string publisherHandle,map message)\n\nPublish a message on the topic associated with this publisher.", publish_callback);
            sim::registerScriptCallbackFunction("simROS2.createClient@ROS2", "string clientHandle=simROS2.createClient(string serviceName,string serviceType)\n\nCreate a service client.", createClient_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownClient@ROS2", "simROS2.shutdownClient(string clientHandle)\n\nShutdown the service client.", shutdownClient_callback);
            sim::registerScriptCallbackFunction("simROS2.clientTreatUInt8ArrayAsString@ROS2", "simROS2.clientTreatUInt8ArrayAsString(string clientHandle)\n\nAfter calling this function, this service client will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", clientTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.waitForService@ROS2", "bool result=simROS2.waitForService(string clientHandle,float timeout)\n\nWait for the service associated with this service client.", waitForService_callback);
            sim::registerScriptCallbackFunction("simROS2.call@ROS2", "map result=simROS2.call(string clientHandle,map request)\n\nCall the service associated with this service client.", call_callback);
            sim::registerScriptCallbackFunction("simROS2.createService@ROS2", "string serviceHandle=simROS2.createService(string serviceName,string serviceType,string serviceCallback)\n\nCreate a service.", createService_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownService@ROS2", "simROS2.shutdownService(string serviceHandle)\n\nShutdown the service.", shutdownService_callback);
            sim::registerScriptCallbackFunction("simROS2.serviceTreatUInt8ArrayAsString@ROS2", "simROS2.serviceTreatUInt8ArrayAsString(string serviceHandle)\n\nAfter calling this function, this service will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", serviceTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.createActionClient@ROS2", "string actionClientHandle=simROS2.createActionClient(string actionName,string actionType,string goalResponseCallback,string feedbackCallback,string resultCallback)\n\nCreate a action client.", createActionClient_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownActionClient@ROS2", "simROS2.shutdownActionClient(string actionClientHandle)\n\nShutdown the action client.", shutdownActionClient_callback);
            sim::registerScriptCallbackFunction("simROS2.actionClientTreatUInt8ArrayAsString@ROS2", "simROS2.actionClientTreatUInt8ArrayAsString(string actionClientHandle)\n\nAfter calling this function, this action client will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", actionClientTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.sendGoal@ROS2", "bool success=simROS2.sendGoal(string actionClientHandle,map goal)\n\nSend a goal using the specified action client.", sendGoal_callback);
            sim::registerScriptCallbackFunction("simROS2.cancelLastGoal@ROS2", "bool success=simROS2.cancelLastGoal(string actionClientHandle)\n\nCancel last submitted goal on the specified action client.", cancelLastGoal_callback);
            sim::registerScriptCallbackFunction("simROS2.createActionServer@ROS2", "string actionServerHandle=simROS2.createActionServer(string actionName,string actionType,string handleGoalCallback,string handleCancelCallback,string handleAcceptedCallback)\n\nCreate an action server.", createActionServer_callback);
            sim::registerScriptCallbackFunction("simROS2.shutdownActionServer@ROS2", "simROS2.shutdownActionServer(string actionServerHandle)\n\nShutdown the action server.", shutdownActionServer_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerTreatUInt8ArrayAsString@ROS2", "simROS2.actionServerTreatUInt8ArrayAsString(string actionServerHandle)\n\nAfter calling this function, this action server will treat uint8 arrays as string. Using strings should be in general much faster that using int arrays in Lua.", actionServerTreatUInt8ArrayAsString_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerPublishFeedback@ROS2", "simROS2.actionServerPublishFeedback(string actionServerHandle,string goalUUID,map feedback)\n\nSend an update about the progress of this goal. This must be only called when the goal is executing. If execution of a goal is deferred then", actionServerPublishFeedback_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionAbort@ROS2", "simROS2.actionServerActionAbort(string actionServerHandle,string goalUUID,map result)\n\nIndicate that a goal could not be reached and has been aborted. Only call this if the goal was executing but cannot be completed. This is a terminal state, no more methods should be called on a goal after this is called.", actionServerActionAbort_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionSucceed@ROS2", "simROS2.actionServerActionSucceed(string actionServerHandle,string goalUUID,map result)\n\nIndicate that a goal has succeeded. Only call this if the goal is executing and has reached the desired final state. This is a terminal state, no more methods should be called on a goal after this is called.", actionServerActionSucceed_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionCanceled@ROS2", "simROS2.actionServerActionCanceled(string actionServerHandle,string goalUUID,map result)\n\nIndicate that a goal has been canceled. Only call this if the goal is executing or pending, but has been canceled. This is a terminal state, no more methods should be called on a goal after this is called.", actionServerActionCanceled_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionExecute@ROS2", "simROS2.actionServerActionExecute(string actionServerHandle,string goalUUID)\n\nIndicate that the server is starting to execute a goal. Only call this if the goal is pending.", actionServerActionExecute_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionIsCanceling@ROS2", "bool result=simROS2.actionServerActionIsCanceling(string actionServerHandle,string goalUUID)\n\nCheck if there is a cancel request", actionServerActionIsCanceling_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionIsActive@ROS2", "bool result=simROS2.actionServerActionIsActive(string actionServerHandle,string goalUUID)\n\nCheck if goal is pending or executing", actionServerActionIsActive_callback);
            sim::registerScriptCallbackFunction("simROS2.actionServerActionIsExecuting@ROS2", "bool result=simROS2.actionServerActionIsExecuting(string actionServerHandle,string goalUUID)\n\nCheck if the goal is executing", actionServerActionIsExecuting_callback);
            sim::registerScriptCallbackFunction("simROS2.sendTransform@ROS2", "simROS2.sendTransform(map transform)\n\nPublish a TF transformation between frames.", sendTransform_callback);
            sim::registerScriptCallbackFunction("simROS2.sendTransforms@ROS2", "simROS2.sendTransforms(map transforms)\n\nPublish several TF transformations between frames.", sendTransforms_callback);
            sim::registerScriptCallbackFunction("simROS2.imageTransportCreateSubscription@ROS2", "string subscriptionHandle=simROS2.imageTransportCreateSubscription(string topicName,string topicCallback,int queueSize=1)\n\nCreate a subscription using ImageTransport.", imageTransportCreateSubscription_callback);
            sim::registerScriptCallbackFunction("simROS2.imageTransportShutdownSubscription@ROS2", "simROS2.imageTransportShutdownSubscription(string subscriptionHandle)\n\nShutdown the subscription using ImageTransport.", imageTransportShutdownSubscription_callback);
            sim::registerScriptCallbackFunction("simROS2.imageTransportCreatePublisher@ROS2", "string publisherHandle=simROS2.imageTransportCreatePublisher(string topicName,int queueSize=1)\n\nCreate a publisher using ImageTransport.", imageTransportCreatePublisher_callback);
            sim::registerScriptCallbackFunction("simROS2.imageTransportShutdownPublisher@ROS2", "simROS2.imageTransportShutdownPublisher(string publisherHandle)\n\nShutdown the publisher using ImageTransport.", imageTransportShutdownPublisher_callback);
            sim::registerScriptCallbackFunction("simROS2.imageTransportPublish@ROS2", "simROS2.imageTransportPublish(string publisherHandle,string data,int width,int height,string frame_id)\n\nPublish a message on the topic associated with this publisher using ImageTransport.", imageTransportPublish_callback);
            sim::registerScriptCallbackFunction("simROS2.getTime@ROS2", "map time=simROS2.getTime(int clock_type=sim_ros2_clock_ros)\n\nReturn the current time according to the specified clock.", getTime_callback);
            sim::registerScriptCallbackFunction("simROS2.getParamString@ROS2", "bool exists,string value=simROS2.getParamString(string name,string defaultValue=\"\")\n\nRetrieve a string parameter from the ROS Parameter Server.", getParamString_callback);
            sim::registerScriptCallbackFunction("simROS2.getParamInt@ROS2", "bool exists,int value=simROS2.getParamInt(string name,int defaultValue=0)\n\nRetrieve an integer parameter from the ROS Parameter Server.", getParamInt_callback);
            sim::registerScriptCallbackFunction("simROS2.getParamDouble@ROS2", "bool exists,float value=simROS2.getParamDouble(string name,float defaultValue=0.0)\n\nRetrieve a double parameter from the ROS Parameter Server.", getParamDouble_callback);
            sim::registerScriptCallbackFunction("simROS2.getParamBool@ROS2", "bool exists,bool value=simROS2.getParamBool(string name,bool defaultValue=false)\n\nRetrieve a boolean parameter from the ROS Parameter Server.", getParamBool_callback);
            sim::registerScriptCallbackFunction("simROS2.setParamString@ROS2", "simROS2.setParamString(string name,string value)\n\nSet a string parameter in the ROS Parameter Server.", setParamString_callback);
            sim::registerScriptCallbackFunction("simROS2.setParamInt@ROS2", "simROS2.setParamInt(string name,int value)\n\nSet a integer parameter in the ROS Parameter Server.", setParamInt_callback);
            sim::registerScriptCallbackFunction("simROS2.setParamDouble@ROS2", "simROS2.setParamDouble(string name,float value)\n\nSet a double parameter in the ROS Parameter Server.", setParamDouble_callback);
            sim::registerScriptCallbackFunction("simROS2.setParamBool@ROS2", "simROS2.setParamBool(string name,bool value)\n\nSet a boolean parameter in the ROS Parameter Server.", setParamBool_callback);
            sim::registerScriptCallbackFunction("simROS2.hasParam@ROS2", "bool exists=simROS2.hasParam(string name)\n\nCheck wether a parameter exists in the ROS Parameter Server.", hasParam_callback);
            sim::registerScriptCallbackFunction("simROS2.deleteParam@ROS2", "simROS2.deleteParam(string name)\n\nDelete a parameter in the ROS Parameter Server.", deleteParam_callback);
            sim::registerScriptCallbackFunction("simROS2.createInterface@ROS2", "map result=simROS2.createInterface(string type)\n\nConstruct an interface of the specified type, initialized with the default values.", createInterface_callback);
            sim::registerScriptCallbackFunction("simROS2.getInterfaceConstants@ROS2", "map result=simROS2.getInterfaceConstants(string type)\n\nGet an object with the constants defined in the specified interface.", getInterfaceConstants_callback);
            sim::registerScriptCallbackFunction("simROS2.supportedInterfaces@ROS2", "string[] result=simROS2.supportedInterfaces()\n\nRetrieve a list of supported interfaces.", supportedInterfaces_callback);

#include "lua_calltips.cpp"
        }
        catch(std::exception &ex)
        {
            throw sim::exception("Initialization failed (registerScriptStuff): %s", ex.what());
        }
    }
    catch(sim::exception& ex)
    {
        sim::addLog(sim_verbosity_errors, ex.what());
        return false;
    }
    return true;
}

const char* clock_type_string(clock_type x)
{
    switch(x)
    {
        case sim_ros2_clock_ros: return "sim_ros2_clock_ros";
        case sim_ros2_clock_system: return "sim_ros2_clock_system";
        case sim_ros2_clock_steady: return "sim_ros2_clock_steady";
        default: return "???";
    }
}

const char* action_result_code_string(action_result_code x)
{
    switch(x)
    {
        case sim_ros2_action_result_code_succeeded: return "sim_ros2_action_result_code_succeeded";
        case sim_ros2_action_result_code_aborted: return "sim_ros2_action_result_code_aborted";
        case sim_ros2_action_result_code_canceled: return "sim_ros2_action_result_code_canceled";
        case sim_ros2_action_result_code_unknown: return "sim_ros2_action_result_code_unknown";
        default: return "???";
    }
}

const char* goal_response_string(goal_response x)
{
    switch(x)
    {
        case sim_ros2_goal_response_reject: return "sim_ros2_goal_response_reject";
        case sim_ros2_goal_response_accept_and_execute: return "sim_ros2_goal_response_accept_and_execute";
        case sim_ros2_goal_response_accept_and_defer: return "sim_ros2_goal_response_accept_and_defer";
        default: return "???";
    }
}

const char* cancel_response_string(cancel_response x)
{
    switch(x)
    {
        case sim_ros2_cancel_response_reject: return "sim_ros2_cancel_response_reject";
        case sim_ros2_cancel_response_accept: return "sim_ros2_cancel_response_accept";
        default: return "???";
    }
}

createSubscription_in::createSubscription_in()
{
    queueSize = 1;
}

createSubscription_out::createSubscription_out()
{
}

void createSubscription_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createSubscription_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createSubscription";

    createSubscription_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createSubscription_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 3)
            throw sim::exception("not enough arguments");
        if(numArgs > 4)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createSubscription_callback: reading input argument 1 \"topicName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (topicName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createSubscription_callback: reading input argument 2 \"topicType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (topicType): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("createSubscription_callback: reading input argument 3 \"topicCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (topicCallback): %s", ex.what());
            }
        }

        if(numArgs >= 4)
        {
            addStubsDebugLog("createSubscription_callback: reading input argument 4 \"queueSize\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.queueSize));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 4 (queueSize): %s", ex.what());
            }
        }


        addStubsDebugLog("createSubscription_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createSubscription_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createSubscription_callback: calling callback (createSubscription)");
        simExtROS2_createSubscription(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createSubscription_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createSubscription_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createSubscription_callback: writing output argument 1 \"subscriptionHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.subscriptionHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (subscriptionHandle): %s", ex.what());
        }

        addStubsDebugLog("createSubscription_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createSubscription_callback: finished");
}

shutdownSubscription_in::shutdownSubscription_in()
{
}

shutdownSubscription_out::shutdownSubscription_out()
{
}

void shutdownSubscription_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownSubscription_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownSubscription";

    shutdownSubscription_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownSubscription_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownSubscription_callback: reading input argument 1 \"subscriptionHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.subscriptionHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (subscriptionHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownSubscription_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownSubscription_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownSubscription_callback: calling callback (shutdownSubscription)");
        simExtROS2_shutdownSubscription(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownSubscription_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownSubscription_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownSubscription_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownSubscription_callback: finished");
}

subscriptionTreatUInt8ArrayAsString_in::subscriptionTreatUInt8ArrayAsString_in()
{
}

subscriptionTreatUInt8ArrayAsString_out::subscriptionTreatUInt8ArrayAsString_out()
{
}

void subscriptionTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.subscriptionTreatUInt8ArrayAsString";

    subscriptionTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    subscriptionTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: reading input argument 1 \"subscriptionHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.subscriptionHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (subscriptionHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: calling callback (subscriptionTreatUInt8ArrayAsString)");
        simExtROS2_subscriptionTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("subscriptionTreatUInt8ArrayAsString_callback: finished");
}

createPublisher_in::createPublisher_in()
{
    queueSize = 1;
    latch = false;
}

createPublisher_out::createPublisher_out()
{
}

void createPublisher_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createPublisher_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createPublisher";

    createPublisher_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createPublisher_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 4)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createPublisher_callback: reading input argument 1 \"topicName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (topicName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createPublisher_callback: reading input argument 2 \"topicType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (topicType): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("createPublisher_callback: reading input argument 3 \"queueSize\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.queueSize));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (queueSize): %s", ex.what());
            }
        }

        if(numArgs >= 4)
        {
            addStubsDebugLog("createPublisher_callback: reading input argument 4 \"latch\" (bool)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.latch));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 4 (latch): %s", ex.what());
            }
        }


        addStubsDebugLog("createPublisher_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createPublisher_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createPublisher_callback: calling callback (createPublisher)");
        simExtROS2_createPublisher(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createPublisher_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createPublisher_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createPublisher_callback: writing output argument 1 \"publisherHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.publisherHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (publisherHandle): %s", ex.what());
        }

        addStubsDebugLog("createPublisher_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createPublisher_callback: finished");
}

shutdownPublisher_in::shutdownPublisher_in()
{
}

shutdownPublisher_out::shutdownPublisher_out()
{
}

void shutdownPublisher_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownPublisher_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownPublisher";

    shutdownPublisher_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownPublisher_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownPublisher_callback: reading input argument 1 \"publisherHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.publisherHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (publisherHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownPublisher_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownPublisher_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownPublisher_callback: calling callback (shutdownPublisher)");
        simExtROS2_shutdownPublisher(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownPublisher_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownPublisher_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownPublisher_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownPublisher_callback: finished");
}

publisherTreatUInt8ArrayAsString_in::publisherTreatUInt8ArrayAsString_in()
{
}

publisherTreatUInt8ArrayAsString_out::publisherTreatUInt8ArrayAsString_out()
{
}

void publisherTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.publisherTreatUInt8ArrayAsString";

    publisherTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    publisherTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: reading input argument 1 \"publisherHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.publisherHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (publisherHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: calling callback (publisherTreatUInt8ArrayAsString)");
        simExtROS2_publisherTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("publisherTreatUInt8ArrayAsString_callback: finished");
}

publish_in::publish_in()
{
}

publish_out::publish_out()
{
}

void publish_callback(SScriptCallBack *p)
{
    addStubsDebugLog("publish_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.publish";

    publish_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    publish_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("publish_callback: reading input argument 1 \"publisherHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.publisherHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (publisherHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("publish_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("publish_callback: calling callback (publish)");
        simExtROS2_publish(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("publish_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("publish_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("publish_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("publish_callback: finished");
}

createClient_in::createClient_in()
{
}

createClient_out::createClient_out()
{
}

void createClient_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createClient_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createClient";

    createClient_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createClient_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createClient_callback: reading input argument 1 \"serviceName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (serviceName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createClient_callback: reading input argument 2 \"serviceType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (serviceType): %s", ex.what());
            }
        }


        addStubsDebugLog("createClient_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createClient_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createClient_callback: calling callback (createClient)");
        simExtROS2_createClient(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createClient_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createClient_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createClient_callback: writing output argument 1 \"clientHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.clientHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (clientHandle): %s", ex.what());
        }

        addStubsDebugLog("createClient_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createClient_callback: finished");
}

shutdownClient_in::shutdownClient_in()
{
}

shutdownClient_out::shutdownClient_out()
{
}

void shutdownClient_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownClient_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownClient";

    shutdownClient_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownClient_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownClient_callback: reading input argument 1 \"clientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.clientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (clientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownClient_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownClient_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownClient_callback: calling callback (shutdownClient)");
        simExtROS2_shutdownClient(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownClient_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownClient_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownClient_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownClient_callback: finished");
}

clientTreatUInt8ArrayAsString_in::clientTreatUInt8ArrayAsString_in()
{
}

clientTreatUInt8ArrayAsString_out::clientTreatUInt8ArrayAsString_out()
{
}

void clientTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.clientTreatUInt8ArrayAsString";

    clientTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    clientTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: reading input argument 1 \"clientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.clientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (clientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: calling callback (clientTreatUInt8ArrayAsString)");
        simExtROS2_clientTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("clientTreatUInt8ArrayAsString_callback: finished");
}

waitForService_in::waitForService_in()
{
}

waitForService_out::waitForService_out()
{
}

void waitForService_callback(SScriptCallBack *p)
{
    addStubsDebugLog("waitForService_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.waitForService";

    waitForService_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    waitForService_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("waitForService_callback: reading input argument 1 \"clientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.clientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (clientHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("waitForService_callback: reading input argument 2 \"timeout\" (float)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.timeout));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (timeout): %s", ex.what());
            }
        }


        addStubsDebugLog("waitForService_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("waitForService_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("waitForService_callback: calling callback (waitForService)");
        simExtROS2_waitForService(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("waitForService_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("waitForService_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("waitForService_callback: writing output argument 1 \"result\" (bool)...");
        try
        {
            writeToStack(out_args.result, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (result): %s", ex.what());
        }

        addStubsDebugLog("waitForService_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("waitForService_callback: finished");
}

call_in::call_in()
{
}

call_out::call_out()
{
}

void call_callback(SScriptCallBack *p)
{
    addStubsDebugLog("call_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.call";

    call_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    call_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("call_callback: reading input argument 1 \"clientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.clientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (clientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("call_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("call_callback: calling callback (call)");
        simExtROS2_call(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("call_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);


        // write output arguments to stack


        addStubsDebugLog("call_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("call_callback: finished");
}

createService_in::createService_in()
{
}

createService_out::createService_out()
{
}

void createService_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createService_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createService";

    createService_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createService_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 3)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createService_callback: reading input argument 1 \"serviceName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (serviceName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createService_callback: reading input argument 2 \"serviceType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (serviceType): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("createService_callback: reading input argument 3 \"serviceCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (serviceCallback): %s", ex.what());
            }
        }


        addStubsDebugLog("createService_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createService_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createService_callback: calling callback (createService)");
        simExtROS2_createService(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createService_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createService_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createService_callback: writing output argument 1 \"serviceHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.serviceHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (serviceHandle): %s", ex.what());
        }

        addStubsDebugLog("createService_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createService_callback: finished");
}

shutdownService_in::shutdownService_in()
{
}

shutdownService_out::shutdownService_out()
{
}

void shutdownService_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownService_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownService";

    shutdownService_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownService_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownService_callback: reading input argument 1 \"serviceHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (serviceHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownService_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownService_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownService_callback: calling callback (shutdownService)");
        simExtROS2_shutdownService(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownService_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownService_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownService_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownService_callback: finished");
}

serviceTreatUInt8ArrayAsString_in::serviceTreatUInt8ArrayAsString_in()
{
}

serviceTreatUInt8ArrayAsString_out::serviceTreatUInt8ArrayAsString_out()
{
}

void serviceTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.serviceTreatUInt8ArrayAsString";

    serviceTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    serviceTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: reading input argument 1 \"serviceHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.serviceHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (serviceHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: calling callback (serviceTreatUInt8ArrayAsString)");
        simExtROS2_serviceTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("serviceTreatUInt8ArrayAsString_callback: finished");
}

createActionClient_in::createActionClient_in()
{
}

createActionClient_out::createActionClient_out()
{
}

void createActionClient_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createActionClient_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createActionClient";

    createActionClient_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createActionClient_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 5)
            throw sim::exception("not enough arguments");
        if(numArgs > 5)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createActionClient_callback: reading input argument 1 \"actionName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createActionClient_callback: reading input argument 2 \"actionType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (actionType): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("createActionClient_callback: reading input argument 3 \"goalResponseCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalResponseCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (goalResponseCallback): %s", ex.what());
            }
        }

        if(numArgs >= 4)
        {
            addStubsDebugLog("createActionClient_callback: reading input argument 4 \"feedbackCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.feedbackCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 4 (feedbackCallback): %s", ex.what());
            }
        }

        if(numArgs >= 5)
        {
            addStubsDebugLog("createActionClient_callback: reading input argument 5 \"resultCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.resultCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 5 (resultCallback): %s", ex.what());
            }
        }


        addStubsDebugLog("createActionClient_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createActionClient_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createActionClient_callback: calling callback (createActionClient)");
        simExtROS2_createActionClient(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createActionClient_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createActionClient_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createActionClient_callback: writing output argument 1 \"actionClientHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.actionClientHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (actionClientHandle): %s", ex.what());
        }

        addStubsDebugLog("createActionClient_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createActionClient_callback: finished");
}

shutdownActionClient_in::shutdownActionClient_in()
{
}

shutdownActionClient_out::shutdownActionClient_out()
{
}

void shutdownActionClient_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownActionClient_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownActionClient";

    shutdownActionClient_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownActionClient_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownActionClient_callback: reading input argument 1 \"actionClientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionClientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionClientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownActionClient_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownActionClient_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownActionClient_callback: calling callback (shutdownActionClient)");
        simExtROS2_shutdownActionClient(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownActionClient_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownActionClient_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownActionClient_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownActionClient_callback: finished");
}

actionClientTreatUInt8ArrayAsString_in::actionClientTreatUInt8ArrayAsString_in()
{
}

actionClientTreatUInt8ArrayAsString_out::actionClientTreatUInt8ArrayAsString_out()
{
}

void actionClientTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionClientTreatUInt8ArrayAsString";

    actionClientTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionClientTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: reading input argument 1 \"actionClientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionClientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionClientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: calling callback (actionClientTreatUInt8ArrayAsString)");
        simExtROS2_actionClientTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionClientTreatUInt8ArrayAsString_callback: finished");
}

sendGoal_in::sendGoal_in()
{
}

sendGoal_out::sendGoal_out()
{
}

void sendGoal_callback(SScriptCallBack *p)
{
    addStubsDebugLog("sendGoal_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.sendGoal";

    sendGoal_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    sendGoal_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("sendGoal_callback: reading input argument 1 \"actionClientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionClientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionClientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("sendGoal_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("sendGoal_callback: calling callback (sendGoal)");
        simExtROS2_sendGoal(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("sendGoal_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("sendGoal_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("sendGoal_callback: writing output argument 1 \"success\" (bool)...");
        try
        {
            writeToStack(out_args.success, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (success): %s", ex.what());
        }

        addStubsDebugLog("sendGoal_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("sendGoal_callback: finished");
}

cancelLastGoal_in::cancelLastGoal_in()
{
}

cancelLastGoal_out::cancelLastGoal_out()
{
}

void cancelLastGoal_callback(SScriptCallBack *p)
{
    addStubsDebugLog("cancelLastGoal_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.cancelLastGoal";

    cancelLastGoal_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    cancelLastGoal_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("cancelLastGoal_callback: reading input argument 1 \"actionClientHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionClientHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionClientHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("cancelLastGoal_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("cancelLastGoal_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("cancelLastGoal_callback: calling callback (cancelLastGoal)");
        simExtROS2_cancelLastGoal(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("cancelLastGoal_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("cancelLastGoal_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("cancelLastGoal_callback: writing output argument 1 \"success\" (bool)...");
        try
        {
            writeToStack(out_args.success, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (success): %s", ex.what());
        }

        addStubsDebugLog("cancelLastGoal_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("cancelLastGoal_callback: finished");
}

createActionServer_in::createActionServer_in()
{
}

createActionServer_out::createActionServer_out()
{
}

void createActionServer_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createActionServer_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createActionServer";

    createActionServer_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createActionServer_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 5)
            throw sim::exception("not enough arguments");
        if(numArgs > 5)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createActionServer_callback: reading input argument 1 \"actionName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("createActionServer_callback: reading input argument 2 \"actionType\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionType));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (actionType): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("createActionServer_callback: reading input argument 3 \"handleGoalCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.handleGoalCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (handleGoalCallback): %s", ex.what());
            }
        }

        if(numArgs >= 4)
        {
            addStubsDebugLog("createActionServer_callback: reading input argument 4 \"handleCancelCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.handleCancelCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 4 (handleCancelCallback): %s", ex.what());
            }
        }

        if(numArgs >= 5)
        {
            addStubsDebugLog("createActionServer_callback: reading input argument 5 \"handleAcceptedCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.handleAcceptedCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 5 (handleAcceptedCallback): %s", ex.what());
            }
        }


        addStubsDebugLog("createActionServer_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createActionServer_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createActionServer_callback: calling callback (createActionServer)");
        simExtROS2_createActionServer(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createActionServer_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createActionServer_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("createActionServer_callback: writing output argument 1 \"actionServerHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.actionServerHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (actionServerHandle): %s", ex.what());
        }

        addStubsDebugLog("createActionServer_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createActionServer_callback: finished");
}

shutdownActionServer_in::shutdownActionServer_in()
{
}

shutdownActionServer_out::shutdownActionServer_out()
{
}

void shutdownActionServer_callback(SScriptCallBack *p)
{
    addStubsDebugLog("shutdownActionServer_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.shutdownActionServer";

    shutdownActionServer_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    shutdownActionServer_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("shutdownActionServer_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("shutdownActionServer_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownActionServer_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("shutdownActionServer_callback: calling callback (shutdownActionServer)");
        simExtROS2_shutdownActionServer(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("shutdownActionServer_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("shutdownActionServer_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("shutdownActionServer_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("shutdownActionServer_callback: finished");
}

actionServerTreatUInt8ArrayAsString_in::actionServerTreatUInt8ArrayAsString_in()
{
}

actionServerTreatUInt8ArrayAsString_out::actionServerTreatUInt8ArrayAsString_out()
{
}

void actionServerTreatUInt8ArrayAsString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerTreatUInt8ArrayAsString";

    actionServerTreatUInt8ArrayAsString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerTreatUInt8ArrayAsString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: calling callback (actionServerTreatUInt8ArrayAsString)");
        simExtROS2_actionServerTreatUInt8ArrayAsString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerTreatUInt8ArrayAsString_callback: finished");
}

actionServerPublishFeedback_in::actionServerPublishFeedback_in()
{
}

actionServerPublishFeedback_out::actionServerPublishFeedback_out()
{
}

void actionServerPublishFeedback_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerPublishFeedback_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerPublishFeedback";

    actionServerPublishFeedback_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerPublishFeedback_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerPublishFeedback_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerPublishFeedback_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerPublishFeedback_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("actionServerPublishFeedback_callback: calling callback (actionServerPublishFeedback)");
        simExtROS2_actionServerPublishFeedback(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerPublishFeedback_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerPublishFeedback_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerPublishFeedback_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerPublishFeedback_callback: finished");
}

actionServerActionAbort_in::actionServerActionAbort_in()
{
}

actionServerActionAbort_out::actionServerActionAbort_out()
{
}

void actionServerActionAbort_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionAbort_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionAbort";

    actionServerActionAbort_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionAbort_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionAbort_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionAbort_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionAbort_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("actionServerActionAbort_callback: calling callback (actionServerActionAbort)");
        simExtROS2_actionServerActionAbort(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionAbort_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionAbort_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerActionAbort_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionAbort_callback: finished");
}

actionServerActionSucceed_in::actionServerActionSucceed_in()
{
}

actionServerActionSucceed_out::actionServerActionSucceed_out()
{
}

void actionServerActionSucceed_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionSucceed_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionSucceed";

    actionServerActionSucceed_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionSucceed_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionSucceed_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionSucceed_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionSucceed_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("actionServerActionSucceed_callback: calling callback (actionServerActionSucceed)");
        simExtROS2_actionServerActionSucceed(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionSucceed_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionSucceed_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerActionSucceed_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionSucceed_callback: finished");
}

actionServerActionCanceled_in::actionServerActionCanceled_in()
{
}

actionServerActionCanceled_out::actionServerActionCanceled_out()
{
}

void actionServerActionCanceled_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionCanceled_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionCanceled";

    actionServerActionCanceled_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionCanceled_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionCanceled_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionCanceled_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionCanceled_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("actionServerActionCanceled_callback: calling callback (actionServerActionCanceled)");
        simExtROS2_actionServerActionCanceled(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionCanceled_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionCanceled_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerActionCanceled_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionCanceled_callback: finished");
}

actionServerActionExecute_in::actionServerActionExecute_in()
{
}

actionServerActionExecute_out::actionServerActionExecute_out()
{
}

void actionServerActionExecute_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionExecute_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionExecute";

    actionServerActionExecute_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionExecute_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionExecute_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionExecute_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionExecute_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionExecute_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionServerActionExecute_callback: calling callback (actionServerActionExecute)");
        simExtROS2_actionServerActionExecute(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionExecute_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionExecute_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("actionServerActionExecute_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionExecute_callback: finished");
}

actionServerActionIsCanceling_in::actionServerActionIsCanceling_in()
{
}

actionServerActionIsCanceling_out::actionServerActionIsCanceling_out()
{
}

void actionServerActionIsCanceling_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionIsCanceling_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionIsCanceling";

    actionServerActionIsCanceling_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionIsCanceling_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionIsCanceling_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionIsCanceling_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionIsCanceling_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsCanceling_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionServerActionIsCanceling_callback: calling callback (actionServerActionIsCanceling)");
        simExtROS2_actionServerActionIsCanceling(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionIsCanceling_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsCanceling_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("actionServerActionIsCanceling_callback: writing output argument 1 \"result\" (bool)...");
        try
        {
            writeToStack(out_args.result, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (result): %s", ex.what());
        }

        addStubsDebugLog("actionServerActionIsCanceling_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionIsCanceling_callback: finished");
}

actionServerActionIsActive_in::actionServerActionIsActive_in()
{
}

actionServerActionIsActive_out::actionServerActionIsActive_out()
{
}

void actionServerActionIsActive_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionIsActive_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionIsActive";

    actionServerActionIsActive_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionIsActive_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionIsActive_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionIsActive_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionIsActive_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsActive_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionServerActionIsActive_callback: calling callback (actionServerActionIsActive)");
        simExtROS2_actionServerActionIsActive(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionIsActive_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsActive_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("actionServerActionIsActive_callback: writing output argument 1 \"result\" (bool)...");
        try
        {
            writeToStack(out_args.result, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (result): %s", ex.what());
        }

        addStubsDebugLog("actionServerActionIsActive_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionIsActive_callback: finished");
}

actionServerActionIsExecuting_in::actionServerActionIsExecuting_in()
{
}

actionServerActionIsExecuting_out::actionServerActionIsExecuting_out()
{
}

void actionServerActionIsExecuting_callback(SScriptCallBack *p)
{
    addStubsDebugLog("actionServerActionIsExecuting_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.actionServerActionIsExecuting";

    actionServerActionIsExecuting_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    actionServerActionIsExecuting_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("actionServerActionIsExecuting_callback: reading input argument 1 \"actionServerHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.actionServerHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (actionServerHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("actionServerActionIsExecuting_callback: reading input argument 2 \"goalUUID\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.goalUUID));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (goalUUID): %s", ex.what());
            }
        }


        addStubsDebugLog("actionServerActionIsExecuting_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsExecuting_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("actionServerActionIsExecuting_callback: calling callback (actionServerActionIsExecuting)");
        simExtROS2_actionServerActionIsExecuting(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("actionServerActionIsExecuting_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("actionServerActionIsExecuting_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("actionServerActionIsExecuting_callback: writing output argument 1 \"result\" (bool)...");
        try
        {
            writeToStack(out_args.result, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (result): %s", ex.what());
        }

        addStubsDebugLog("actionServerActionIsExecuting_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("actionServerActionIsExecuting_callback: finished");
}

sendTransform_in::sendTransform_in()
{
}

sendTransform_out::sendTransform_out()
{
}

void sendTransform_callback(SScriptCallBack *p)
{
    addStubsDebugLog("sendTransform_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.sendTransform";

    sendTransform_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    sendTransform_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 0)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack


        addStubsDebugLog("sendTransform_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("sendTransform_callback: calling callback (sendTransform)");
        simExtROS2_sendTransform(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("sendTransform_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("sendTransform_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("sendTransform_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("sendTransform_callback: finished");
}

sendTransforms_in::sendTransforms_in()
{
}

sendTransforms_out::sendTransforms_out()
{
}

void sendTransforms_callback(SScriptCallBack *p)
{
    addStubsDebugLog("sendTransforms_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.sendTransforms";

    sendTransforms_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    sendTransforms_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 0)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack


        addStubsDebugLog("sendTransforms_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);


        addStubsDebugLog("sendTransforms_callback: calling callback (sendTransforms)");
        simExtROS2_sendTransforms(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("sendTransforms_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("sendTransforms_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("sendTransforms_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("sendTransforms_callback: finished");
}

imageTransportCreateSubscription_in::imageTransportCreateSubscription_in()
{
    queueSize = 1;
}

imageTransportCreateSubscription_out::imageTransportCreateSubscription_out()
{
}

void imageTransportCreateSubscription_callback(SScriptCallBack *p)
{
    addStubsDebugLog("imageTransportCreateSubscription_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.imageTransportCreateSubscription";

    imageTransportCreateSubscription_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    imageTransportCreateSubscription_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 3)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("imageTransportCreateSubscription_callback: reading input argument 1 \"topicName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (topicName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("imageTransportCreateSubscription_callback: reading input argument 2 \"topicCallback\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicCallback));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (topicCallback): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("imageTransportCreateSubscription_callback: reading input argument 3 \"queueSize\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.queueSize));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (queueSize): %s", ex.what());
            }
        }


        addStubsDebugLog("imageTransportCreateSubscription_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportCreateSubscription_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("imageTransportCreateSubscription_callback: calling callback (imageTransportCreateSubscription)");
        simExtROS2_imageTransportCreateSubscription(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("imageTransportCreateSubscription_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportCreateSubscription_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("imageTransportCreateSubscription_callback: writing output argument 1 \"subscriptionHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.subscriptionHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (subscriptionHandle): %s", ex.what());
        }

        addStubsDebugLog("imageTransportCreateSubscription_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("imageTransportCreateSubscription_callback: finished");
}

imageTransportShutdownSubscription_in::imageTransportShutdownSubscription_in()
{
}

imageTransportShutdownSubscription_out::imageTransportShutdownSubscription_out()
{
}

void imageTransportShutdownSubscription_callback(SScriptCallBack *p)
{
    addStubsDebugLog("imageTransportShutdownSubscription_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.imageTransportShutdownSubscription";

    imageTransportShutdownSubscription_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    imageTransportShutdownSubscription_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("imageTransportShutdownSubscription_callback: reading input argument 1 \"subscriptionHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.subscriptionHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (subscriptionHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("imageTransportShutdownSubscription_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportShutdownSubscription_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("imageTransportShutdownSubscription_callback: calling callback (imageTransportShutdownSubscription)");
        simExtROS2_imageTransportShutdownSubscription(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("imageTransportShutdownSubscription_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportShutdownSubscription_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("imageTransportShutdownSubscription_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("imageTransportShutdownSubscription_callback: finished");
}

imageTransportCreatePublisher_in::imageTransportCreatePublisher_in()
{
    queueSize = 1;
}

imageTransportCreatePublisher_out::imageTransportCreatePublisher_out()
{
}

void imageTransportCreatePublisher_callback(SScriptCallBack *p)
{
    addStubsDebugLog("imageTransportCreatePublisher_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.imageTransportCreatePublisher";

    imageTransportCreatePublisher_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    imageTransportCreatePublisher_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("imageTransportCreatePublisher_callback: reading input argument 1 \"topicName\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.topicName));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (topicName): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("imageTransportCreatePublisher_callback: reading input argument 2 \"queueSize\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.queueSize));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (queueSize): %s", ex.what());
            }
        }


        addStubsDebugLog("imageTransportCreatePublisher_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportCreatePublisher_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("imageTransportCreatePublisher_callback: calling callback (imageTransportCreatePublisher)");
        simExtROS2_imageTransportCreatePublisher(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("imageTransportCreatePublisher_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportCreatePublisher_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("imageTransportCreatePublisher_callback: writing output argument 1 \"publisherHandle\" (std::string)...");
        try
        {
            writeToStack(out_args.publisherHandle, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (publisherHandle): %s", ex.what());
        }

        addStubsDebugLog("imageTransportCreatePublisher_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("imageTransportCreatePublisher_callback: finished");
}

imageTransportShutdownPublisher_in::imageTransportShutdownPublisher_in()
{
}

imageTransportShutdownPublisher_out::imageTransportShutdownPublisher_out()
{
}

void imageTransportShutdownPublisher_callback(SScriptCallBack *p)
{
    addStubsDebugLog("imageTransportShutdownPublisher_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.imageTransportShutdownPublisher";

    imageTransportShutdownPublisher_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    imageTransportShutdownPublisher_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("imageTransportShutdownPublisher_callback: reading input argument 1 \"publisherHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.publisherHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (publisherHandle): %s", ex.what());
            }
        }


        addStubsDebugLog("imageTransportShutdownPublisher_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportShutdownPublisher_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("imageTransportShutdownPublisher_callback: calling callback (imageTransportShutdownPublisher)");
        simExtROS2_imageTransportShutdownPublisher(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("imageTransportShutdownPublisher_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportShutdownPublisher_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("imageTransportShutdownPublisher_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("imageTransportShutdownPublisher_callback: finished");
}

imageTransportPublish_in::imageTransportPublish_in()
{
}

imageTransportPublish_out::imageTransportPublish_out()
{
}

void imageTransportPublish_callback(SScriptCallBack *p)
{
    addStubsDebugLog("imageTransportPublish_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.imageTransportPublish";

    imageTransportPublish_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    imageTransportPublish_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 5)
            throw sim::exception("not enough arguments");
        if(numArgs > 5)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("imageTransportPublish_callback: reading input argument 1 \"publisherHandle\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.publisherHandle));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (publisherHandle): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("imageTransportPublish_callback: reading input argument 2 \"data\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.data));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (data): %s", ex.what());
            }
        }

        if(numArgs >= 3)
        {
            addStubsDebugLog("imageTransportPublish_callback: reading input argument 3 \"width\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.width));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 3 (width): %s", ex.what());
            }
        }

        if(numArgs >= 4)
        {
            addStubsDebugLog("imageTransportPublish_callback: reading input argument 4 \"height\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.height));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 4 (height): %s", ex.what());
            }
        }

        if(numArgs >= 5)
        {
            addStubsDebugLog("imageTransportPublish_callback: reading input argument 5 \"frame_id\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.frame_id));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 5 (frame_id): %s", ex.what());
            }
        }


        addStubsDebugLog("imageTransportPublish_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportPublish_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("imageTransportPublish_callback: calling callback (imageTransportPublish)");
        simExtROS2_imageTransportPublish(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("imageTransportPublish_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("imageTransportPublish_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("imageTransportPublish_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("imageTransportPublish_callback: finished");
}

getTime_in::getTime_in()
{
    clock_type = sim_ros2_clock_ros;
}

getTime_out::getTime_out()
{
}

void getTime_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getTime_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getTime";

    getTime_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getTime_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 0)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getTime_callback: reading input argument 1 \"clock_type\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.clock_type));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (clock_type): %s", ex.what());
            }
        }


        addStubsDebugLog("getTime_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getTime_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getTime_callback: calling callback (getTime)");
        simExtROS2_getTime(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getTime_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getTime_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("getTime_callback: writing output argument 1 \"time\" (sim_ros2_time)...");
        try
        {
            writeToStack(out_args.time, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (time): %s", ex.what());
        }

        addStubsDebugLog("getTime_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getTime_callback: finished");
}

getParamString_in::getParamString_in()
{
    defaultValue = "";
}

getParamString_out::getParamString_out()
{
    value = "";
}

void getParamString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getParamString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getParamString";

    getParamString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getParamString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getParamString_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("getParamString_callback: reading input argument 2 \"defaultValue\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.defaultValue));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (defaultValue): %s", ex.what());
            }
        }


        addStubsDebugLog("getParamString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getParamString_callback: calling callback (getParamString)");
        simExtROS2_getParamString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getParamString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("getParamString_callback: writing output argument 1 \"exists\" (bool)...");
        try
        {
            writeToStack(out_args.exists, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (exists): %s", ex.what());
        }
        addStubsDebugLog("getParamString_callback: writing output argument 2 \"value\" (std::string)...");
        try
        {
            writeToStack(out_args.value, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 2 (value): %s", ex.what());
        }

        addStubsDebugLog("getParamString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getParamString_callback: finished");
}

getParamInt_in::getParamInt_in()
{
    defaultValue = 0;
}

getParamInt_out::getParamInt_out()
{
    value = 0;
}

void getParamInt_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getParamInt_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getParamInt";

    getParamInt_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getParamInt_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getParamInt_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("getParamInt_callback: reading input argument 2 \"defaultValue\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.defaultValue));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (defaultValue): %s", ex.what());
            }
        }


        addStubsDebugLog("getParamInt_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamInt_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getParamInt_callback: calling callback (getParamInt)");
        simExtROS2_getParamInt(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getParamInt_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamInt_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("getParamInt_callback: writing output argument 1 \"exists\" (bool)...");
        try
        {
            writeToStack(out_args.exists, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (exists): %s", ex.what());
        }
        addStubsDebugLog("getParamInt_callback: writing output argument 2 \"value\" (int)...");
        try
        {
            writeToStack(out_args.value, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 2 (value): %s", ex.what());
        }

        addStubsDebugLog("getParamInt_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getParamInt_callback: finished");
}

getParamDouble_in::getParamDouble_in()
{
    defaultValue = 0.0;
}

getParamDouble_out::getParamDouble_out()
{
    value = 0.0;
}

void getParamDouble_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getParamDouble_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getParamDouble";

    getParamDouble_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getParamDouble_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getParamDouble_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("getParamDouble_callback: reading input argument 2 \"defaultValue\" (double)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.defaultValue));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (defaultValue): %s", ex.what());
            }
        }


        addStubsDebugLog("getParamDouble_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamDouble_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getParamDouble_callback: calling callback (getParamDouble)");
        simExtROS2_getParamDouble(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getParamDouble_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamDouble_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("getParamDouble_callback: writing output argument 1 \"exists\" (bool)...");
        try
        {
            writeToStack(out_args.exists, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (exists): %s", ex.what());
        }
        addStubsDebugLog("getParamDouble_callback: writing output argument 2 \"value\" (double)...");
        try
        {
            writeToStack(out_args.value, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 2 (value): %s", ex.what());
        }

        addStubsDebugLog("getParamDouble_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getParamDouble_callback: finished");
}

getParamBool_in::getParamBool_in()
{
    defaultValue = false;
}

getParamBool_out::getParamBool_out()
{
    value = false;
}

void getParamBool_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getParamBool_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getParamBool";

    getParamBool_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getParamBool_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getParamBool_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("getParamBool_callback: reading input argument 2 \"defaultValue\" (bool)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.defaultValue));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (defaultValue): %s", ex.what());
            }
        }


        addStubsDebugLog("getParamBool_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamBool_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getParamBool_callback: calling callback (getParamBool)");
        simExtROS2_getParamBool(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getParamBool_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getParamBool_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("getParamBool_callback: writing output argument 1 \"exists\" (bool)...");
        try
        {
            writeToStack(out_args.exists, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (exists): %s", ex.what());
        }
        addStubsDebugLog("getParamBool_callback: writing output argument 2 \"value\" (bool)...");
        try
        {
            writeToStack(out_args.value, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 2 (value): %s", ex.what());
        }

        addStubsDebugLog("getParamBool_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getParamBool_callback: finished");
}

setParamString_in::setParamString_in()
{
}

setParamString_out::setParamString_out()
{
}

void setParamString_callback(SScriptCallBack *p)
{
    addStubsDebugLog("setParamString_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.setParamString";

    setParamString_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    setParamString_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("setParamString_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("setParamString_callback: reading input argument 2 \"value\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.value));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (value): %s", ex.what());
            }
        }


        addStubsDebugLog("setParamString_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamString_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("setParamString_callback: calling callback (setParamString)");
        simExtROS2_setParamString(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("setParamString_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamString_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("setParamString_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("setParamString_callback: finished");
}

setParamInt_in::setParamInt_in()
{
}

setParamInt_out::setParamInt_out()
{
}

void setParamInt_callback(SScriptCallBack *p)
{
    addStubsDebugLog("setParamInt_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.setParamInt";

    setParamInt_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    setParamInt_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("setParamInt_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("setParamInt_callback: reading input argument 2 \"value\" (int)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.value));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (value): %s", ex.what());
            }
        }


        addStubsDebugLog("setParamInt_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamInt_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("setParamInt_callback: calling callback (setParamInt)");
        simExtROS2_setParamInt(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("setParamInt_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamInt_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("setParamInt_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("setParamInt_callback: finished");
}

setParamDouble_in::setParamDouble_in()
{
}

setParamDouble_out::setParamDouble_out()
{
}

void setParamDouble_callback(SScriptCallBack *p)
{
    addStubsDebugLog("setParamDouble_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.setParamDouble";

    setParamDouble_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    setParamDouble_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("setParamDouble_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("setParamDouble_callback: reading input argument 2 \"value\" (double)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.value));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (value): %s", ex.what());
            }
        }


        addStubsDebugLog("setParamDouble_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamDouble_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("setParamDouble_callback: calling callback (setParamDouble)");
        simExtROS2_setParamDouble(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("setParamDouble_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamDouble_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("setParamDouble_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("setParamDouble_callback: finished");
}

setParamBool_in::setParamBool_in()
{
}

setParamBool_out::setParamBool_out()
{
}

void setParamBool_callback(SScriptCallBack *p)
{
    addStubsDebugLog("setParamBool_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.setParamBool";

    setParamBool_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    setParamBool_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 2)
            throw sim::exception("not enough arguments");
        if(numArgs > 2)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("setParamBool_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }

        if(numArgs >= 2)
        {
            addStubsDebugLog("setParamBool_callback: reading input argument 2 \"value\" (bool)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.value));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 2 (value): %s", ex.what());
            }
        }


        addStubsDebugLog("setParamBool_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamBool_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("setParamBool_callback: calling callback (setParamBool)");
        simExtROS2_setParamBool(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("setParamBool_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("setParamBool_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("setParamBool_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("setParamBool_callback: finished");
}

hasParam_in::hasParam_in()
{
}

hasParam_out::hasParam_out()
{
}

void hasParam_callback(SScriptCallBack *p)
{
    addStubsDebugLog("hasParam_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.hasParam";

    hasParam_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    hasParam_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("hasParam_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }


        addStubsDebugLog("hasParam_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("hasParam_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("hasParam_callback: calling callback (hasParam)");
        simExtROS2_hasParam(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("hasParam_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("hasParam_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("hasParam_callback: writing output argument 1 \"exists\" (bool)...");
        try
        {
            writeToStack(out_args.exists, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (exists): %s", ex.what());
        }

        addStubsDebugLog("hasParam_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("hasParam_callback: finished");
}

deleteParam_in::deleteParam_in()
{
}

deleteParam_out::deleteParam_out()
{
}

void deleteParam_callback(SScriptCallBack *p)
{
    addStubsDebugLog("deleteParam_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.deleteParam";

    deleteParam_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    deleteParam_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("deleteParam_callback: reading input argument 1 \"name\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.name));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (name): %s", ex.what());
            }
        }


        addStubsDebugLog("deleteParam_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("deleteParam_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("deleteParam_callback: calling callback (deleteParam)");
        simExtROS2_deleteParam(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("deleteParam_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("deleteParam_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack


        addStubsDebugLog("deleteParam_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("deleteParam_callback: finished");
}

createInterface_in::createInterface_in()
{
}

createInterface_out::createInterface_out()
{
}

void createInterface_callback(SScriptCallBack *p)
{
    addStubsDebugLog("createInterface_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.createInterface";

    createInterface_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    createInterface_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("createInterface_callback: reading input argument 1 \"type\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.type));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (type): %s", ex.what());
            }
        }


        addStubsDebugLog("createInterface_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("createInterface_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("createInterface_callback: calling callback (createInterface)");
        simExtROS2_createInterface(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("createInterface_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);


        // write output arguments to stack


        addStubsDebugLog("createInterface_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("createInterface_callback: finished");
}

getInterfaceConstants_in::getInterfaceConstants_in()
{
}

getInterfaceConstants_out::getInterfaceConstants_out()
{
}

void getInterfaceConstants_callback(SScriptCallBack *p)
{
    addStubsDebugLog("getInterfaceConstants_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.getInterfaceConstants";

    getInterfaceConstants_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    getInterfaceConstants_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 1)
            throw sim::exception("not enough arguments");
        if(numArgs > 1)
            throw sim::exception("too many arguments");

        // read input arguments from stack

        if(numArgs >= 1)
        {
            addStubsDebugLog("getInterfaceConstants_callback: reading input argument 1 \"type\" (std::string)...");
            try
            {
                sim::moveStackItemToTop(p->stackID, 0);
                readFromStack(p->stackID, &(in_args.type));
            }
            catch(std::exception &ex)
            {
                throw sim::exception("read in arg 1 (type): %s", ex.what());
            }
        }


        addStubsDebugLog("getInterfaceConstants_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("getInterfaceConstants_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("getInterfaceConstants_callback: calling callback (getInterfaceConstants)");
        simExtROS2_getInterfaceConstants(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("getInterfaceConstants_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);


        // write output arguments to stack


        addStubsDebugLog("getInterfaceConstants_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("getInterfaceConstants_callback: finished");
}

supportedInterfaces_in::supportedInterfaces_in()
{
}

supportedInterfaces_out::supportedInterfaces_out()
{
}

void supportedInterfaces_callback(SScriptCallBack *p)
{
    addStubsDebugLog("supportedInterfaces_callback: reading input arguments...");
    addStubsDebugStackDump(p->stackID);

    const char *cmd = "simROS2.supportedInterfaces";

    supportedInterfaces_in in_args;
    if(p)
    {
        std::memcpy(&in_args._, p, sizeof(SScriptCallBack));
    }
    supportedInterfaces_out out_args;

    try
    {
        // check argument count

        int numArgs = sim::getStackSize(p->stackID);
        if(numArgs < 0)
            throw sim::exception("not enough arguments");
        if(numArgs > 0)
            throw sim::exception("too many arguments");

        // read input arguments from stack


        addStubsDebugLog("supportedInterfaces_callback: stack content after reading input arguments:");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("supportedInterfaces_callback: clearing stack content after reading input arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        addStubsDebugLog("supportedInterfaces_callback: calling callback (supportedInterfaces)");
        simExtROS2_supportedInterfaces(&in_args, &out_args);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
    }

    try
    {
        addStubsDebugLog("supportedInterfaces_callback: writing output arguments...");
        addStubsDebugStackDump(p->stackID);

        addStubsDebugLog("supportedInterfaces_callback: clearing stack content before writing output arguments");
        // clear stack
        sim::popStackItem(p->stackID, 0);


        // write output arguments to stack

        addStubsDebugLog("supportedInterfaces_callback: writing output argument 1 \"result\" (std::vector< std::string >)...");
        try
        {
            writeToStack(out_args.result, p->stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("write out arg 1 (result): %s", ex.what());
        }

        addStubsDebugLog("supportedInterfaces_callback: stack content after writing output arguments:");
        addStubsDebugStackDump(p->stackID);
    }
    catch(std::exception &ex)
    {
        sim::setLastError(cmd, ex.what());
        // clear stack
        try { sim::popStackItem(p->stackID, 0); } catch(...) {}
    }

    addStubsDebugLog("supportedInterfaces_callback: finished");
}

subscriptionCallback_in::subscriptionCallback_in()
{
}

subscriptionCallback_out::subscriptionCallback_out()
{
}

bool subscriptionCallback(int scriptId, const char *func, subscriptionCallback_in *in_args, subscriptionCallback_out *out_args)
{
    addStubsDebugLog("subscriptionCallback: writing input arguments...");

    int stackID = -1;

    try
    {
        stackID = sim::createStack();

        // write input arguments to stack


        addStubsDebugLog("subscriptionCallback: wrote input arguments:");
        addStubsDebugStackDump(stackID);

        sim::callScriptFunctionEx(scriptId, func, stackID);

        // read output arguments from stack

        addStubsDebugLog("subscriptionCallback: reading output arguments...");


        addStubsDebugLog("subscriptionCallback: stack content after reading output arguments:");
        addStubsDebugStackDump(stackID);

        sim::releaseStack(stackID);
        stackID = -1;
    }
    catch(std::exception &ex)
    {
        if(stackID != -1)
            try { sim::releaseStack(stackID); } catch(...) {}
        sim::setLastError(func, ex.what());
        return false;
    }

    addStubsDebugLog("subscriptionCallback: finished");

    return true;
}

imageTransportCallback_in::imageTransportCallback_in()
{
}

imageTransportCallback_out::imageTransportCallback_out()
{
}

bool imageTransportCallback(int scriptId, const char *func, imageTransportCallback_in *in_args, imageTransportCallback_out *out_args)
{
    addStubsDebugLog("imageTransportCallback: writing input arguments...");

    int stackID = -1;

    try
    {
        stackID = sim::createStack();

        // write input arguments to stack

        addStubsDebugLog("imageTransportCallback: writing input argument 1 \"data\" (std::string)...");
        try
        {
            writeToStack(in_args->data, stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("writing input argument 1 (data): %s", ex.what());
        }
        addStubsDebugLog("imageTransportCallback: writing input argument 2 \"width\" (int)...");
        try
        {
            writeToStack(in_args->width, stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("writing input argument 2 (width): %s", ex.what());
        }
        addStubsDebugLog("imageTransportCallback: writing input argument 3 \"height\" (int)...");
        try
        {
            writeToStack(in_args->height, stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("writing input argument 3 (height): %s", ex.what());
        }

        addStubsDebugLog("imageTransportCallback: wrote input arguments:");
        addStubsDebugStackDump(stackID);

        sim::callScriptFunctionEx(scriptId, func, stackID);

        // read output arguments from stack

        addStubsDebugLog("imageTransportCallback: reading output arguments...");


        addStubsDebugLog("imageTransportCallback: stack content after reading output arguments:");
        addStubsDebugStackDump(stackID);

        sim::releaseStack(stackID);
        stackID = -1;
    }
    catch(std::exception &ex)
    {
        if(stackID != -1)
            try { sim::releaseStack(stackID); } catch(...) {}
        sim::setLastError(func, ex.what());
        return false;
    }

    addStubsDebugLog("imageTransportCallback: finished");

    return true;
}

actionGoalResponseCallback_in::actionGoalResponseCallback_in()
{
}

actionGoalResponseCallback_out::actionGoalResponseCallback_out()
{
}

bool actionGoalResponseCallback(int scriptId, const char *func, actionGoalResponseCallback_in *in_args, actionGoalResponseCallback_out *out_args)
{
    addStubsDebugLog("actionGoalResponseCallback: writing input arguments...");

    int stackID = -1;

    try
    {
        stackID = sim::createStack();

        // write input arguments to stack

        addStubsDebugLog("actionGoalResponseCallback: writing input argument 1 \"goalID\" (std::string)...");
        try
        {
            writeToStack(in_args->goalID, stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("writing input argument 1 (goalID): %s", ex.what());
        }
        addStubsDebugLog("actionGoalResponseCallback: writing input argument 2 \"accepted\" (bool)...");
        try
        {
            writeToStack(in_args->accepted, stackID);
        }
        catch(std::exception &ex)
        {
            throw sim::exception("writing input argument 2 (accepted): %s", ex.what());
        }

        addStubsDebugLog("actionGoalResponseCallback: wrote input arguments:");
        addStubsDebugStackDump(stackID);

        sim::callScriptFunctionEx(scriptId, func, stackID);

        // read output arguments from stack

        addStubsDebugLog("actionGoalResponseCallback: reading output arguments...");


        addStubsDebugLog("actionGoalResponseCallback: stack content after reading output arguments:");
        addStubsDebugStackDump(stackID);

        sim::releaseStack(stackID);
        stackID = -1;
    }
    catch(std::exception &ex)
    {
        if(stackID != -1)
            try { sim::releaseStack(stackID); } catch(...) {}
        sim::setLastError(func, ex.what());
        return false;
    }

    addStubsDebugLog("actionGoalResponseCallback: finished");

    return true;
}

