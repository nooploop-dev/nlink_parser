
[简体中文](./README.md) | English

<h1 align="center">Protocol Extracter</h1>

<div align="center">

![Logo](http://ftp.nooploop.com/media/image/nooploop.png)

Protocol Extracter is a general data protocol extractor, which decouples protocol implementation and protocol extraction. Users only need to input data and process protocol callback function

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://dev.azure.com/ant-design/ant-design-pro/_build/latest?definitionId=1?branchName=master) ![Test Status](https://img.shields.io/badge/test-passing-brightgreen)


</div>

## Usage

- NProtocolBase , as a general protocol base class, users inherit according to their protocols, fill in frame header, and implement the verification
- NProtocolExtracter, as a protocol extractor, it manages protocols, inputs data, automatically splices data, and extracts protocol frames

Refer to [example.cpp](./example.cpp) to get start

## License

The source code is released under a [BSD 3-Clause license](LICENSE).


