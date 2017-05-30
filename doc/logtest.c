#include <sns.h>


int main( int argc, char **argv ) {
    sns_init();

    SNS_LOG(LOG_NOTICE, "Hello from C");

    sns_end();

    return 0;
}
