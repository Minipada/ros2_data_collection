import Builder
import os
import sys

class ElasticurlTests(Builder.Action):
    def run(self, env):
        elasticurl_path = os.path.join(env.install_dir, 'bin', 'elasticurl_cpp')
        if sys.platform == 'win32':
            elasticurl_path += '.exe'

        # don't run elasticurl if we didn't build it for whatever reason (ex: BYO_CRYPTO)
        if not os.path.exists(elasticurl_path):
            print('{} not found, skipping elasticurl tests'.format(elasticurl_path))
            return

        env.shell.exec(sys.executable, 'crt/aws-c-http/integration-testing/http_client_test.py', elasticurl_path,
                       check=True)
