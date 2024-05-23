#ifndef _PREDEFINE_FUNCTION_RESULT_H_
#define _PREDEFINE_FUNCTION_RESULT_H_

namespace Predefine
{
    class FunctionResult
    {
    public:
        static const int SUCCESS = 0;
        static const int FAILURE = -1;
        static const int RUNNING = 1;
        static const int PREPARE = 2;
        static const int EXECUTE = 3;
        static const int NOT_EXECUTE = 4;
    };
}

#endif
