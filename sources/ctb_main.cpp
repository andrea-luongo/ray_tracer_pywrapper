#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;

void add_rep(uint8_t gray7, uint32_t stride, uint32_t& bits_on, std::vector<uint8_t>& rle)
{
    if (stride == 0)
        return;
    if (gray7 > 0)
    {
        bits_on += stride;
    }
    if (stride > 1)
    {
        gray7 |= uint8_t(0x80);
    }
    rle.push_back(gray7);
    if (stride <= 1)
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


py::array_t<int32_t> RGBA(py::array_t<int32_t>& pic)
{
    py::buffer_info buf1 = pic.request();
    py::array_t<int32_t> result = py::array_t<int32_t>(buf1.size);

    py::buffer_info buf2 = result.request();
    int32_t* ptr1 = (int32_t*)buf1.ptr,
        * ptr2 = (int32_t*)buf2.ptr;
    int X = buf1.shape[0];
    int Y = buf1.shape[1];
    int Z = buf1.shape[2];
    for (size_t idx = 0; idx < X; idx++) {
        for (size_t idy = 0; idy < Y; idy++) {
            for (size_t idz = 0; idz < Z; idz++) {
                ptr2[idx * Y * Z + idy * Z + idz] = ptr1[idx * Y * Z + idy * Z + idz] | (ptr1[idx * Y * Z + idy * Z + idz] << 8);
            }
        }
    }
    // reshape array to match input shape
    result.resize({ X,Y,Z });

    return result;
    
}


py::tuple rle_encode_graymap(py::array_t<uint8_t>& grey_array)
//py::tuple rle_encode_graymap(py::array_t<uint8_t>& grey_array)
{
    uint8_t color(0xff);
    uint32_t stride(0);
    std::vector<uint8_t> rle;
    uint32_t bits_on(0);
    for (int idx = 0; idx < grey_array.size(); idx++)
    {
        uint8_t gray7 = grey_array.at(idx);
        if (gray7 == color)
        {
            stride += uint8_t(1);
        }
        else 
        {
            add_rep(color, stride, bits_on, rle);
            color = gray7;
            stride = 1;
        }
    }
    add_rep(color, stride, bits_on, rle);
    //return py::make_tuple(rle , bits_on);
    return py::make_tuple(py::array_t<uint8_t>(rle.size(), rle.data()), bits_on);
}


PYBIND11_MODULE(ctbConverterPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for .ctb conversion
    )pbdoc";
    m.def("rle_encode_graymap", &rle_encode_graymap);
    m.def("RGBA", &RGBA);
  
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



