#!/bin/bash

/bin/netstat -an | grep LISTEN | grep -c $PORT
