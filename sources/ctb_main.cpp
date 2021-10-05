#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;

void add_rep(uint8_t grey, uint32_t stride, uint8_t color, uint32_t& bits_on, std::vector<uint8_t>& rle)
{
    if (stride == 0)
        return;
    if (grey > 0)
    {
        bits_on += stride;
    }
    else if (grey > 1)
    {
        grey |= uint8_t(0x80);
    }
    rle.push_back(grey);
    if (grey <= 1)
    {
        return;
    }
    else if (stride <= 0x7f)
    {
        rle.push_back(uint8_t(stride));
    }
    else if (stride <= 0x3fff)
    {
        rle.push_back(uint8_t(stride >> 8) | uint8_t(0x80));
        rle.push_back(uint8_t(stride));
    }
    else if (stride <= 0x1fffff)
    {
        rle.push_back(uint8_t(stride >> 16) | uint8_t(0xc0));
        rle.push_back(uint8_t(stride >> 8));
        rle.push_back(uint8_t(stride));
    }
    else if (stride <= 0xfffffff)
    {
        rle.push_back(uint8_t(stride >> 24) | uint8_t(0xe0));
        rle.push_back(uint8_t(stride >> 16));
        rle.push_back(uint8_t(stride >> 8));
        rle.push_back(uint8_t(stride));
    }
}


py::tuple rle_encode_graymap(py::array_t<uint8_t>& grey_array)
{
    uint8_t color = 0xff;
    uint32_t stride = 0;
    std::vector<uint8_t> rle;
    uint32_t bits_on = 0;
    std::cout << grey_array << std::endl;
    std::cout << grey_array.size() << std::endl;
    std::cout << grey_array.dtype() << std::endl;
    for (int idx = 0; idx < grey_array.size(); idx++)
    {
        uint8_t grey = grey_array.at(idx);
        std::cout << grey << std::endl;
        if (grey == color)
        {
            stride += 1;
            std::cout << "stride " << stride << std::endl;
        }
        else 
        {
            add_rep(grey, stride, color, bits_on, rle);
            color = grey;
            stride = 1;

            std::cout << "stride " << stride << std::endl;
            std::cout << "stride " << color << std::endl;
        }
    }
    return py::make_tuple(py::cast(rle) , bits_on);
}

//#     def add_rep(gray7: np.uint8, stride : np.uint) :
//    #         nonlocal bits_on, rle
//#         if stride == 0:
//    #             return
//#         if gray7 > 0:
//    #             bits_on += stride
//#         if stride > 1:
//    #             gray7 |= np.uint8(0x80)
//    #         rle.append(gray7)
//#         if stride <= 1:
//    #             _ = 0
//#         elif stride <= 0x7f:
//    #             rle.append(np.uint8(stride))
//#         elif stride <= 0x3fff:
//    #             rle.append(np.uint8(stride >> 8) | np.uint8(0x80))
//    #             rle.append(np.uint8(stride))
//#         elif stride <= 0x1fffff:
//    #             rle.append(np.uint8(stride >> 16) | np.uint8(0xc0))
//    #             rle.append(np.uint8(stride >> 8))
//    #             rle.append(np.uint8(stride))
//#         elif stride <= 0xfffffff:
//    #             rle.append(np.uint8(stride >> 24) | np.uint8(0xe0))
//    #             rle.append(np.uint8(stride >> 16))
//    #             rle.append(np.uint8(stride >> 8))
//    #             rle.append(np.uint8(stride))
//#
//    #     def build_rle() :
//    #         nonlocal stride, color
//    #         for y in range(size[0]) :
//    #             for x in range(size[1]) :
//    #                 grey7 = grey7_matrix[y, x]
//#                 if grey7 == color:
//    #                     stride += 1
//#                 else:
//    #                     add_rep(color, stride)
//    #                     color = grey7
//    #                     stride = 1


PYBIND11_MODULE(ctbConverterPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for .ctb conversion
    )pbdoc";
    m.def("rle_encode_graymap", &rle_encode_graymap);
  
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



