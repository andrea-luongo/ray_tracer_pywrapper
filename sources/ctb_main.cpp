#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
//#include <BVH.h>
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


py::tuple rle_encode_graymap(py::array_t<uint8_t>& grey_array, py::array_t<uint32_t> pic_size, py::array_t<uint32_t> max_size)
{
    uint8_t color(0xff);
    uint32_t stride(0);
    std::vector<uint8_t> rle;
    uint32_t bits_on(0);

    uint8_t black_color(0);

    uint32_t top_black_stride = max_size.at(1) * floor((max_size.at(0) - pic_size.at(0)) * 0.5);
    uint32_t left_black_stride = floor((max_size.at(1) - pic_size.at(1)) * 0.5);
    uint32_t rigth_black_stride = ceil((max_size.at(1) - pic_size.at(1)) * 0.5);
    uint32_t bottom_black_stride = max_size.at(1) * ceil((max_size.at(0) - pic_size.at(0)) * 0.5);

    color = black_color;
    stride = top_black_stride;
    for (int row = 0; row < pic_size.at(0); row++)
    {
        if (color == black_color) {
            stride += left_black_stride;
        }
        else {
            add_rep(color, stride, bits_on, rle);
            color = black_color;
            stride = left_black_stride;
        }
        for (int col = 0; col < pic_size.at(1); col++)
        {
            uint8_t gray7 = grey_array.at(col + row * pic_size.at(1));
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
        if (color == black_color) {
            stride += rigth_black_stride;
        }
        else {
            add_rep(color, stride, bits_on, rle);
            color = black_color;
            stride = rigth_black_stride;
        }
    }
    //Step 3: add bottom black rows to rle
    if (color == black_color) {
        stride += bottom_black_stride;
    }
    else {
        add_rep(color, stride, bits_on, rle);
        color = black_color;
        stride = bottom_black_stride;
    }
    add_rep(color, stride, bits_on, rle);

    return py::make_tuple(py::array_t<uint8_t>(rle.size(), rle.data()), bits_on);
}


//py::array_t<int32_t> RGBA(py::array_t<int32_t>& pic)
py::tuple RGBA(py::array_t<int32_t>& pic, bool compute_gray)
{
    py::buffer_info buf1 = pic.request();
    py::array_t<int32_t> result = py::array_t<int32_t>(buf1.size);

    py::buffer_info buf2 = result.request();
    int32_t* ptr1 = (int32_t*)buf1.ptr,
        * ptr2 = (int32_t*)buf2.ptr;
    int X = buf1.shape[0];
    int Y = buf1.shape[1];
    int Z = buf1.shape[2];

    py::array_t<uint8_t> gray_result;
    py::buffer_info buf3;
    uint8_t* ptr3 = nullptr;

    if (compute_gray) {
        gray_result = py::array_t<uint8_t>(X * Y);
        buf3 = gray_result.request();
        ptr3 = (uint8_t*)buf3.ptr;

    }

    for (size_t idx = 0; idx < X; idx++) {
        for (size_t idy = 0; idy < Y; idy++) {
            for (size_t idz = 0; idz < Z; idz++) {
                ptr2[idx * Y * Z + idy * Z + idz] = ptr1[idx * Y * Z + idy * Z + idz] | (ptr1[idx * Y * Z + idy * Z + idz] << 8);
            }
            if (compute_gray)
                ptr3[idx * Y + idy] = uint16_t(ptr2[idx * Y * Z + idy * Z + 0] | ptr2[idx * Y * Z + idy * Z + 1] | ptr2[idx * Y * Z + idy * Z + 2]) >> uint16_t(9);
        }
    }
    // reshape array to match input shape
    result.resize({ X,Y,Z });
    if (compute_gray)
        gray_result.resize({ X, Y });

    //return result;
    return py::make_tuple(result, gray_result);

}


PYBIND11_MODULE(ctbConverterPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for .ctb conversion
    )pbdoc";
    m.def("rle_encode_graymap", &rle_encode_graymap, R"pbdoc(
		Optimized RLE encoding of graymap

		:param grey_array: numpy.array of numpy.int8 values
		:param pic_size: numpy.array of numpy.int32 containing the size of the sliced images
		:param max_size: numpy.array of numpy.int32 containing the max size of the Elegoo printer
		:return: 
			- **rle_size**: int
			- **rle_data**: np.array of uint8
			- **bits_on**: uint32
    )pbdoc");
    m.def("RGBA", &RGBA, R"pbdoc(

		:param pic: numpy.array of numpy.int32 values
		:param compute_gray: bool
		:return: 
			- **result**: numpy.array of numpy.int32
			- **gray_result**: numpy.array of numpy.uint8
    )pbdoc");
  
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



