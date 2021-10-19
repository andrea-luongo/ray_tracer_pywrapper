#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;



uint64_t ISO = 0xD800000000000000;
uint64_t ECMA = 0xC96C5795D7870F42;

std::vector<uint64_t> Make_Table(uint64_t poly)
{
    return;
}

std::vector<uint64_t> make_slicing_by_8_table(py::array_t<uint64_t>& t) {
    py::array_t<uint64_t> helper_table({ 8, 256 });
    helper_table[0] = t;
    for (int i = 0; i < 256; i++)
    {
        uint64_t crc = t.at(i);
    }

}
    //helper_table = [np.zeros(256, dtype = np.uint64) for _ in range(8)]
    //helper_table[0] = t
    //for i in range(256) :
    //    crc = t[i]
    //    for j in range(1, 8) :
    //        crc = t[crc & np.uint64(0xff)] ^ (crc >> np.uint64(8))
    //        helper_table[j][i] = crc
    //        return helper_table

std::vector<uint64_t> make_table(uint64_t poly)
{
    int size = 256;
    uint64_t crc;
    std::vector<uint64_t> t(size);
    for (uint64_t i = 0; i < size; i++) 
    {
        crc = i;
        for (int j = 0; j < 8; j++) 
        {
            if ((crc & uint64_t(1)) == uint64_t(1)) 
            {
                crc = (crc >> uint64_t(1)) ^ poly;
            }
            else
            {
                crc >>= uint64_t(1);
            }
        }
        t[i] = crc;
    }
    return t;
}
//
//def Make_Table(poly: np.uint64) :
//    if poly == ISO :
//        return make_slicing_by_8_table(make_table(ISO))[0]
//    elif poly == ECMA :
//        return make_slicing_by_8_table(make_table(ECMA))[0]
//    else :
//        return make_table(poly)




int culo()
{
    return 1;
}



PYBIND11_MODULE(crc64PyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for .ctb conversion
    )pbdoc";
    m.def("culo", &culo);
  
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



