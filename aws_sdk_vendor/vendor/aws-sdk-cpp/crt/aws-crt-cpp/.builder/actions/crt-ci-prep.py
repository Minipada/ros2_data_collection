import Builder

class CrtCiPrep(Builder.Action):
    def run(self, env):
        env.shell.setenv("AWS_TESTING_STS_ROLE_ARN", env.shell.get_secret("aws-c-auth-testing/sts-role-arn"))
        actions = [
            Builder.SetupCrossCICrtEnvironment()
        ]
        return Builder.Script(actions, name='crt-ci-prep')
