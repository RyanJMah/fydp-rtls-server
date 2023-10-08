#pragma once

#include <iostream>
#include <stdarg.h>

#include "Recast.h"

class GL_RecastContext : public rcContext
{
    public:
        // override to print to stdout
	    virtual void doLog(const rcLogCategory category, const char* msg, const int len)
        {
            switch ( category )
            {
                case RC_LOG_PROGRESS:
                {
                    std::cout << "RECAST-INFO: ";
                    break;
                }

                case RC_LOG_WARNING:
                {
                    std::cout << "RECAST-WARNING: ";
                    break;
                }

                case RC_LOG_ERROR:
                {
                    std::cout << "RECAST-ERROR: ";
                    break;
                }

                default:
                {
                    std::cout << "INVALID LOG CATEGORY, ABORTING!!!!" << std::endl;
                    exit(1);
                }
            }

            std::cout << msg << std::endl;
        }
};