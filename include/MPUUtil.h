#ifndef __MPUUTIL_H__
#define __MPUUTIL_H__

class MPUUtil {
    public:
        static MPUUtil* getInstance();
        void write_data();
        void read();
        void setup();
        void wake();
    private:
        MPUUtil();
        MPUUtil(const MPUUtil&) = delete;
        MPUUtil& operator=(const MPUUtil&) = delete;
        static MPUUtil* pInstance;
};


#endif