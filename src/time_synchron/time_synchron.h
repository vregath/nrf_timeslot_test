
#ifndef _TIME_SYNCHRON_H__
#define _TIME_SYNCHRON_H__

namespace  notch {

    class TimeSynchron {

        public:
            void initialize();

            void setIsMaster(bool isMaster) {
                if (mIsStarted) {
                    return;
                }
                mIsMaster = isMaster;
            }

            void start(bool isMaster);
            void stop();
            void triggerSendTimeSynchronStatus() {
                mIsSendTimeSynchronStatus = true;
            }

            void setSynchronized(bool isSynchronized, bool isExtTrigger);

            bool getSynchronized() {
                return mIsSynchronized;
            }

            void sendTimeSynchronStatus();            

            void stopSDRadioSession();
            void startSDRadioSession();

            void doStartSDRadioSession();

        private:
            bool mIsSynchronized;
            bool mIsStarted = false;
            bool mIsMaster = false;
            bool mIsSendTimeSynchronStatus = false;
           
            bool mIsRadioSessionStart = false;

        private:
            void gpioteTriggerInit();
    };

    extern TimeSynchron timeSynchron;

}

#endif
