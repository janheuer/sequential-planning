# sequential-planning
> TODO: description

This project implements a sequential planning approach to the [**asprilo**](<https://potassco.org/asprilo>) framework.

TODO: general idea

## Encodings

All encodings used in the project are incremental encodings.

`./parallel_encodings/` contains encodings from the [**development branch**](<https://github.com/potassco/asprilo-encodings/tree/develop>)  of the asprilo encodings repository.

`./sequential_encodings/` contains modified version of the parallel encodings to work for the sequential approach.

## Instance Format

The input and output formats are the standard [**asprilo format**](<https://github.com/potassco/asprilo/blob/master/docs/specification.md#input-format>).

`./instances/` contains several example instances.

For using the sequential planning mode an additional file with assignments of orders to the robots is required.
(For the example instances these are given in files with an additional postfix `o` in the filename, e.g. for instance `instance/1/1.lp` the assignment is in file `instance/1/1o.lp`)

## Usage

```bash
python sequential.py instance
```

Python (tested with version 2.7) and the python module of [**clingo**](<https://github.com/potassco/clingo>) are required.

To get a list of all available options run:
```bash
python sequential.py --help
```
