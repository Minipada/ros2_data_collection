import Builder
import json
import os
import re
import subprocess
import sys


class AuthCiPrep(Builder.Action):

    def run(self, env):
        env.shell.setenv("AWS_TESTING_COGNITO_IDENTITY", env.shell.get_secret("aws-c-auth-testing/cognito-identity"), quiet=True)

        actions = []

        return Builder.Script(actions, name='auth-ci-prep')
