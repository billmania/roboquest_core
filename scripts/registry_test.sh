#!/usr/bin/env bash

export DOCKER_HOST=tcp://192.168.0.24:2376

#
# Any http request is redirected to https
#
curl --verbose -X GET -L http://registry.q4excellence.com:8079/test
# 1 OK


#
# GET http://registry.q4excellence.com:8079/updater_version.txt
# redirects to https and returns a string
#
curl --verbose -X GET -L http://registry.q4excellence.com:8079/updater_version.txt
# 1 OK

#
# GET http://registry.q4excellence.com:8079/updater.py
# redirects to https and returns the Python script
#
curl --verbose -X GET -L http://registry.q4excellence.com:8079/updater.py
# 1 OK

#
# curl -X GET https://registry.q4excellence.com:5678/v2/_catalog
# returns something very similar to:
# {"repositories":["rq_core","rq_ui"]}
#
curl --verbose -X GET -L http://registry.q4excellence.com:8079/v2/_catalog
curl --verbose -X GET https://registry.q4excellence.com:5678/v2/_catalog
# 1 OK

#
# docker pull registry.q4excellence.com:5678/rq_core
# retrieves the image
#
docker pull registry.q4excellence.com:5678/rq_core
# 1 OK

#
# Python docker module image PULL
#
# 1 OK

#
# test authentication with GET +, HEAD +, PUT 403, POST +, PATCH 403,
# DELETE 403
# docker pull does GET and HEAD
# docker push does GET, HEAD, and PUT
#
curl --verbose -X GET -u push:UpYoursNow https://registry.q4excellence.com:5678/secure/put_test
# 1 OK

#
# docker push registry.q4excellence.com:5678/rq_core
# requires authentication and pushes the image to the registry
#
docker login -u push registry.q4excellence.com:5678
docker push push/registry.q4excellence.com:5678/rq_core
# 1 FAIL

exit 0
