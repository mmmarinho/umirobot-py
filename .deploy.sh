#!/bin/bash
if [ "$TRAVIS_BRANCH" = "main" -a "$TRAVIS_PULL_REQUEST" = "false" ]
then
    python3 -m pip install twine
    python3 -m twine upload dist/*
fi