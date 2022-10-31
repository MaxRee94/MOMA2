#include "player.h"
#include <stdio.h>

void print_error() {
    PyObject *type, *val, *tb;
    PyErr_Fetch(&type, &val, &tb);
    PyObject *pstr = PyObject_Str(val);
    PyObject* errbytes = PyUnicode_AsASCIIString(pstr);
    char* err = PyBytes_AsString(errbytes);
    printf("Python says: ");
    printf(err);
    printf("\n");
    Py_DecRef(type);
    Py_DecRef(val);
    Py_DecRef(tb);
    Py_DecRef(pstr);
    Py_DecRef(errbytes);
}

char* callpython(char* message) {
    // printf("entering c\n");
    // printf("message\n");
    // printf(message);
    // printf("\n");

    // printf("init python\n");
    Py_Initialize();

    // printf("adding to path\n");
    char* myrepl;
    PyObject *sys_path = PySys_GetObject("path");
    if(sys_path == NULL) {
        printf("Error C: Could not obtain sys-path.\n");
        print_error();
    }
    else {
        PyList_Append(sys_path, PyUnicode_FromString("src/PingPong/Python"));
        
        // printf("getting module\n");
        PyObject* myModule = PyImport_ImportModule("CLink");
        if(myModule == NULL) {
            printf("Error C: Could not obtain 'CLink' module.\n");
            print_error();
        }
        else {
            // printf("getting function\n");
            PyObject* myFunction = PyObject_GetAttrString(myModule, "handle_message");
            if(myFunction == NULL) {
                printf("Error C: Could not obtain 'handle_message' function.\n");
                print_error();
            }
            else {
                // printf("preparing arguments\n");
                PyObject* myArgs = PyTuple_Pack(1, PyUnicode_FromString(message));
                if(myArgs == NULL) {
                    printf("Error C: Could not create arguments.\n");
                    print_error();
                }
                else {
                    // printf("calling function\n");
                    PyObject* repl = PyObject_CallObject(myFunction, myArgs);
                    if(repl == NULL) {
                        printf("Error C: Reply is NULL.\n");
                        print_error();
                    }
                    else {
                        // printf("extracting answer\n");
                        Py_ssize_t size;
                        PyObject* mybytes = PyUnicode_AsASCIIString(repl);
                        if(mybytes == NULL) {
                            printf("Error C: Could not convert response from python-unicode to python-bytes.\n");
                            print_error();
                        }
                        else {
                            // printf("extracted bytes\n");
                            myrepl = PyBytes_AsString(mybytes);
                            if(myrepl == NULL) {
                                printf("Error C: Could not convert response from python-bytes to c-string.\n");
                                print_error();
                            }
                        }
                    }

                    Py_DecRef(repl);
                }

                Py_DecRef(myArgs);
            }

            Py_DecRef(myFunction);
        }

        Py_DecRef(myModule);
    }

    return myrepl;
}
